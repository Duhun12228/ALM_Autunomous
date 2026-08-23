/**
 * 로봇으로 나가는 명령 — **fetch 가 존재하는 유일한 파일이다.**
 *
 * 읽기와 쓰기는 서로 다른 경로로 나간다.
 *
 *   읽기  ros-bridge.js ──WS  :8765──▶ foxglove_bridge   구독 전용
 *   쓰기  이 파일        ──HTTP:8081──▶ alm_web_backend   토큰 + 제어권 락
 *
 * foxglove_bridge 쪽은 clientPublish 능력 자체가 빠져 있어 브라우저가 ROS
 * 토픽에 직접 publish 할 방법이 없다. 그래야 cmd_arbiter 의 동작권과
 * command_manager 의 안전 게이팅을 우회할 수 없다.
 *
 * 제어권(락)에 대하여: 서버가 세션 하나에만 쓰기를 허용한다. 브라우저가
 * 크래시하면 release 를 못 보내므로 서버는 리스(기본 15초)로 관리하고,
 * 여기서는 그 1/3 주기로 하트비트를 보낸다.
 */

const TOKEN_KEY = 'alm.backendToken';
const URL_KEY = 'alm.backendUrl';
const HEARTBEAT_MS = 5000;

/** 백엔드 주소. 저장된 값 > ?backend= > 같은 호스트의 8081 */
export function resolveBackendUrl() {
  const stored = sessionStorage.getItem(URL_KEY);
  if (stored) return stored.replace(/\/+$/, '');
  const fromQuery = new URLSearchParams(location.search).get('backend');
  if (fromQuery) return fromQuery.replace(/\/+$/, '');
  const host = location.hostname || 'localhost';
  return `${location.protocol === 'https:' ? 'https' : 'http'}://${host}:8081`;
}

export class CommandError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

export class Commands {
  constructor(alm, { onSessionChange } = {}) {
    this.alm = alm;
    this.base = resolveBackendUrl();
    this.token = sessionStorage.getItem(TOKEN_KEY) || '';
    this.sessionId = '';
    this.heartbeatTimer = null;
    // 화면 갱신이 실패해도 명령 계층의 상태 기계는 계속 굴러가야 한다.
    // (세션은 이미 서버에서 풀렸는데 렌더 예외로 그 사실이 묻히면, 화면은
    //  제어권을 쥔 것처럼 남고 다음 명령마다 409 를 받는다)
    const notify = onSessionChange || (() => {});
    this.onSessionChange = (held, info) => {
      try {
        notify(held, info);
      } catch (error) {
        console.error('[alm] 세션 상태 렌더 실패', error);
      }
    };
    this.healthy = null;      // null = 아직 모름
  }

  // ── 토큰 ────────────────────────────────────────────────────────────
  hasToken() {
    return Boolean(this.token);
  }

  setToken(token) {
    this.token = (token || '').trim();
    if (this.token) sessionStorage.setItem(TOKEN_KEY, this.token);
    else sessionStorage.removeItem(TOKEN_KEY);
  }

  setBaseUrl(url) {
    const clean = (url || '').trim().replace(/\/+$/, '');
    if (!clean) return;
    this.base = clean;
    sessionStorage.setItem(URL_KEY, clean);
  }

  // ── 요청 ────────────────────────────────────────────────────────────
  async request(method, path, body, { silent = false } = {}) {
    if (!this.token) {
      throw new CommandError(401, '백엔드 토큰이 없습니다. 설정에서 입력하세요.');
    }
    const headers = { Authorization: `Bearer ${this.token}` };
    if (this.sessionId) headers['X-ALM-Session'] = this.sessionId;
    if (body !== undefined) headers['Content-Type'] = 'application/json';

    let response;
    try {
      response = await fetch(this.base + path, {
        method,
        headers,
        body: body === undefined ? undefined : JSON.stringify(body),
      });
    } catch (error) {
      this.healthy = false;
      throw new CommandError(0, `백엔드에 연결할 수 없습니다 (${this.base}) — ${error.message}`);
    }

    this.healthy = response.status < 500;
    let payload = null;
    try {
      payload = await response.json();
    } catch {
      payload = null;
    }

    if (!response.ok) {
      const message = payload?.error || `HTTP ${response.status}`;
      // 제어권을 잃었으면 하트비트를 멈춘다 — 계속 두드려봐야 409만 쌓인다.
      if (response.status === 409 && this.sessionId && path.startsWith('/api/session')) {
        this._dropSession();
      }
      if (!silent) this._report(response.status, message);
      throw new CommandError(response.status, message);
    }
    return payload ?? {};
  }

  _report(status, message) {
    const title = status === 401 ? '인증 실패'
      : status === 409 ? '지금은 할 수 없습니다'
        : status === 0 ? '백엔드 연결 실패' : '명령 실패';
    this.alm?.toast(title, message, status === 409 ? 'warning' : 'error');
    this.alm?.addLog(status === 409 ? 'WARN' : 'ERROR', 'alm_web_backend', message);
  }

  // ── 제어권 ──────────────────────────────────────────────────────────
  get hasControl() {
    return Boolean(this.sessionId);
  }

  async acquireControl(label) {
    const result = await this.request('POST', '/api/session/acquire', { label: label || '' });
    this.sessionId = result.session_id;
    this._startHeartbeat();
    this.onSessionChange(true, result);
    return result;
  }

  async releaseControl() {
    if (!this.sessionId) return;
    try {
      await this.request('POST', '/api/session/release', {}, { silent: true });
    } catch {
      /* 반납 실패는 리스 만료로 어차피 정리된다 */
    }
    this._dropSession();
  }

  /**
   * 탭이 닫힐 때의 반납. 일반 요청과 달리 keepalive 로 보낸다.
   *
   * navigator.sendBeacon 이 아닌 이유: beacon 은 커스텀 헤더를 못 붙여서
   * Authorization/X-ALM-Session 이 빠지고 서버가 401 로 버린다.
   * 이것도 실패할 수 있지만 그때는 서버 리스가 TTL 뒤에 알아서 정리한다.
   */
  releaseControlOnUnload() {
    if (!this.sessionId) return;
    fetch(`${this.base}/api/session/release`, {
      method: 'POST',
      keepalive: true,
      headers: {
        Authorization: `Bearer ${this.token}`,
        'X-ALM-Session': this.sessionId,
        'Content-Type': 'application/json',
      },
      body: '{}',
    }).catch(() => { /* 닫히는 중이라 결과를 볼 수 없다 */ });
  }

  _dropSession() {
    this.sessionId = '';
    clearInterval(this.heartbeatTimer);
    this.heartbeatTimer = null;
    this.onSessionChange(false, null);
  }

  _startHeartbeat() {
    clearInterval(this.heartbeatTimer);
    this.heartbeatTimer = setInterval(async () => {
      try {
        await this.request('POST', '/api/session/heartbeat', {}, { silent: true });
      } catch (error) {
        if (error.status === 409 || error.status === 401) {
          this.alm?.toast('제어권이 해제되었습니다', error.message, 'warning');
          this._dropSession();
        }
        // 네트워크 오류(status 0)는 잠깐 끊긴 것일 수 있으니 세션을 버리지 않는다.
        // 서버 쪽 리스가 만료되면 다음 명령이 409 로 알려준다.
      }
    }, HEARTBEAT_MS);
  }

  // ── 엔드포인트 ──────────────────────────────────────────────────────
  health() { return this.request('GET', '/api/health', undefined, { silent: true }); }
  limits() { return this.request('GET', '/api/limits', undefined, { silent: true }); }
  sessionStatus() { return this.request('GET', '/api/session', undefined, { silent: true }); }

  estop() { return this.request('POST', '/api/estop', {}); }
  releaseEstop(reason) { return this.request('POST', '/api/estop/release', { reason }); }

  startMapping(map, overwrite = false) {
    return this.request('POST', '/api/mapping/start', { map, overwrite });
  }
  stopMapping() { return this.request('POST', '/api/mapping/stop', {}); }
  saveMap() { return this.request('POST', '/api/mapping/save', {}); }

  // ── 측위 ────────────────────────────────────────────────────────────
  // map 을 생략하면 서버가 active.yaml 의 활성 맵을 쓴다. 화면이 맵 이름을
  // 지어내 보내지 않는 편이 낫다 — 활성 맵의 진실은 서버에 있고, 여기서
  // 캐시한 값을 보내면 그 사이 바뀐 경우 엉뚱한 맵으로 측위가 뜬다.
  startLocalization(options = {}) {
    return this.request('POST', '/api/localization/start', options);
  }
  stopLocalization() { return this.request('POST', '/api/localization/stop', {}); }
  localizationStatus() {
    return this.request('GET', '/api/localization', undefined, { silent: true });
  }
  localizationLog(since = 0) {
    return this.request('GET', `/api/localization/log?since=${since}`,
      undefined, { silent: true });
  }

  // ── 자율주행 ────────────────────────────────────────────────────────
  // 스택 기동/종료와 목표 전송은 **다른 층이다.** start 는 Nav2 프로세스를
  // 띄우는 것이고(수십 초), goal 은 이미 떠 있는 Nav2 에 목표를 주는 것이다
  // (수십 ms). 화면에서 '주행 시작' 버튼 하나로 보이더라도 섞으면 안 된다 —
  // 목표를 보낼 때마다 스택을 다시 띄우면 초기 정합을 매번 다시 한다.
  startNavigationStack(options = {}) {
    return this.request('POST', '/api/navigation/start', options);
  }
  stopNavigationStack() { return this.request('POST', '/api/navigation/stop', {}); }
  navigationStatus() {
    return this.request('GET', '/api/navigation', undefined, { silent: true });
  }
  navigationLog(since = 0) {
    return this.request('GET', `/api/navigation/log?since=${since}`,
      undefined, { silent: true });
  }

  /** points: [{x, y, yaw_deg}]. 하나면 단일 목표, 여럿이면 웨이포인트 미션. */
  sendGoal(points) {
    return this.request('POST', '/api/navigation/goal', { points });
  }
  // 일시정지는 Nav2 목표를 **취소**하고 남은 목록을 서버가 기억하는 것이다.
  // 재개하면 남은 목록으로 새 목표를 보낸다 — 세운 자리에서 이어붙이지 않는다.
  pauseNavigation() { return this.request('POST', '/api/navigation/pause', {}); }
  resumeNavigation() { return this.request('POST', '/api/navigation/resume', {}); }
  cancelNavigation() { return this.request('POST', '/api/navigation/cancel', {}); }

  createMap(name, label, notes) {
    return this.request('POST', '/api/maps', { name, label, notes });
  }
  setActiveMap(name) { return this.request('PUT', '/api/maps/active', { name }); }

  runPcd2Pgm(options) { return this.request('POST', '/api/jobs/pcd2pgm', options); }
  runFpfh(options) { return this.request('POST', '/api/jobs/fpfh', options); }
  job(id, since = 0) {
    return this.request('GET', `/api/jobs/${encodeURIComponent(id)}?since=${since}`,
      undefined, { silent: true });
  }

  /**
   * 작업이 끝날 때까지 폴링하며 새 줄을 onLine 으로 흘린다.
   *
   * SSE 대신 폴링인 이유: EventSource 는 커스텀 헤더를 못 붙여서 Bearer 토큰을
   * 쿼리스트링에 실어야 한다(접속 로그에 남는다). 백엔드에 SSE 엔드포인트가
   * 있긴 하지만 그건 curl 디버깅용이고, 화면은 폴링이 더 단순하고 튼튼하다.
   */
  async followJob(id, { onLine, intervalMs = 600, timeoutMs = 900000 } = {}) {
    let since = 0;
    const deadline = Date.now() + timeoutMs;
    for (;;) {
      const snapshot = await this.job(id, since);
      for (const line of snapshot.lines || []) onLine?.(line, snapshot);
      since = snapshot.line_count ?? since;
      if (snapshot.state !== 'running') return snapshot;
      if (Date.now() > deadline) {
        throw new CommandError(504, '작업이 너무 오래 걸립니다 — 로봇 쪽 로그를 확인하세요.');
      }
      await new Promise((resolve) => setTimeout(resolve, intervalMs));
    }
  }
}
