(() => {
  'use strict';

  const $ = (selector, root = document) => root.querySelector(selector);
  const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
  const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

  // 템플릿 리터럴로 innerHTML 을 조립하는 곳에는 전부 이걸 통과시킨다.
  // 목업일 땐 모든 문자열이 하드코딩이라 무해했지만, /rosout 로그와 MCU 의
  // fault_text, 서버가 준 맵 이름이 들어오는 순간 인젝션 경로가 된다.
  // 속성값(data-*, value=)에도 쓰므로 따옴표까지 막는다.
  const esc = (value) => String(value ?? '')
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;').replace(/'/g, '&#39;');

  // 라이브(실 로봇 연동) 여부.
  // 모듈 스크립트(src/main.js)는 defer 라 이 IIFE 본문보다 늦게, 그러나
  // DOMContentLoaded(=init) 보다는 먼저 실행된다. 그래서 상수로 잡으면 항상
  // false 가 된다 — init() 시점에 읽도록 함수로 둔다.
  const isLive = () => window.ALM_LIVE === true;
  const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  const nowTime = () => new Date().toLocaleTimeString('ko-KR', { hour12: false });
  const fmtDuration = (seconds) => `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;

  const TAB_META = {
    mapping: ['OPERATION / 01', 'SLAM 매핑'],
    navigation: ['OPERATION / 02', '측위 · 자율주행'],
    manual: ['OPERATION / 03', '수동주행'],
    monitoring: ['OPERATION / 04', '시스템 모니터링'],
  };

  // 측위 방식은 이 브랜치에서 FPFH+TEASER++ 하나로 확정됐다 (커밋 bbc6dad).
  // 예전의 방식 A/B/C 선택은 제거했다 — 고를 것이 없다.
  const PIPELINE_DESC = 'FPFH + TEASER++ 전역 초기화 · GICP 지역 정합';

  const state = {
    tab: 'mapping',
    // 맵 목록은 지어내지 않는다. /alm/map_inventory 를 받기 전까지는
    // '아직 모른다'(mapInventory === null) 로 두고, 화면도 그렇게 말한다.
    activeMap: '—',
    maps: [],
    mapInventory: null,
    estop: false,
    hasControl: true,
    systemState: 'IDLE',
    mappingElapsed: 0,
    mappingTimer: null,
    mappingSaved: false,
    // 서버(/api/health)가 보는 slam 프로세스 상태. 이 탭에서 시작하지 않은
    // 매핑도 알아야 하므로 systemState 와 별개로 둔다.
    slamRunning: null,
    // /livox/lidar 가 재생본인지. null = 아직 모름 (실측과 섞으면 안 된다)
    lidarReplay: null,
    pointCount: 128420,
    mappingSteps: [],
    logs: [],
    // rmse/overlap 은 [GICP] 로그에서 뽑은 **실측**이다. 예전에는 'fitness' 라는
    // 이름으로 0.087 을 지어내 그리고 있었다.
    localization: {
      state: 'IDLE', frame: 0, frameTotal: 10, attempts: 0,
      matches: null, matchesNeeded: null, accepted: 0, needed: 0,
      rmse: null, overlap: null, pose: null, map: '', running: null, external: false,
    },
    // 정합 로그는 전역 logs 와 **다른 버퍼**다. 120줄짜리 전역 버퍼에 섞으면
    // fast_lio 수다에 밀려 정합 줄이 몇 초 만에 사라진다.
    localizationLogs: [],
    locLogSource: 'ros',
    locLogFollow: true,
    locLogOffset: 0,
    locLogTimer: null,
    waypoints: [],
    // 목표 헤딩 드래그 중의 임시 상태 {from, to}. 안 끌고 있으면 null.
    goalDrag: null,
    addWaypoint: false,
    manualPose: false,
    mapScale: 1,
    // nav 는 /api/navigation 의 캐시다 (자율주행 절 도입부 참조).
    //   serverState  서버가 준 원본 상태 (idle/pending/active/paused/...)
    //   state        화면 라벨 (IDLE/RUNNING/PAUSED/COMPLETED/FAILED)
    //   distance0    단일 목표의 진척률 분모. 첫 피드백의 남은거리로 잡는다
    //   mode         조작자가 고른 요청 모드. 실제 모드는 state.driveMode
    nav: {
      state: 'IDLE', serverState: '', kind: '', progress: 0,
      current: 0, total: 0, distance: null, distance0: null, eta: null,
      estimate: null, recoveries: 0, message: '',
      stackRunning: false, stackExternal: false,
      ready: false, actionReady: false, tfReady: false,
      pollTimer: null, mode: 'auto',
    },
    alarms: [],
    // 수동주행은 rpm/조향각 직접 조작이다 (twist 아님 — 해당 절 도입부 참조).
    //   rpm/steer   지금 보내고 있는 명령
    //   actualRpm   /mcu/command 가 실제로 낸 값 (null = 아직 모름)
    //   heldKeys    키보드 데드맨. 마지막 키를 떼야 멈춘다
    manual: {
      enabled: false, mode: 'normal', multiplier: 0.5, command: null,
      rpm: 0, steer: 0, actualRpm: null, streamTimer: null, heldKeys: new Set(),
    },
    metrics: { cpu: 34, ram: 52, gpu: 28, temp: 45.2, battery: 85, power: 8.7 },
    chart: [],
  };

  const STEP_BASE = [
    ['sensor', '센서 기동', 'alm_sensors lidar.launch.py'],
    ['engine', '매핑 엔진 기동', 'FAST-LIO / SC-LIO-SAM'],
    ['scanning', '차량 주행 및 매핑', '외부 조이스틱 사용'],
    ['save_pcd', '3D PCD 저장', '/map_save'],
    ['pcd2pgm', 'PCD → PGM 변환', '2D Navigation map'],
    ['fpfh_db', 'FPFH 측위 DB 생성', 'fpfh_map_builder'],
  ];

  function resetMappingSteps() {
    state.mappingSteps = STEP_BASE.map(([key, title, detail]) => ({
      key, title, detail, status: 'pending',
    }));
    renderMappingSteps();
  }

  function toast(title, message = '', type = 'info') {
    const node = document.createElement('div');
    node.className = `toast ${type}`;
    node.innerHTML = `<i></i><div><strong>${esc(title)}</strong><small>${esc(message)}</small></div>`;
    $('#toastStack').appendChild(node);
    setTimeout(() => node.remove(), 3800);
  }

  function openModal(content) {
    $('#modal').innerHTML = content;
    $('#modalBackdrop').classList.remove('hidden');
    $$('[data-close-modal]', $('#modal')).forEach((button) => button.addEventListener('click', closeModal));
  }

  function closeModal() {
    $('#modalBackdrop').classList.add('hidden');
    $('#modal').innerHTML = '';
  }

  /**
   * 버튼을 '처리 중' 상태로 만든다.
   *
   * ⚠ 자식 요소가 있는 버튼의 내용은 건드리지 않는다. 예전 구현은 무조건
   * `textContent = label` 로 덮었는데, 그러면 안쪽 구조가 통째로 사라진다.
   * 실제로 두 곳이 깨져 있었다:
   *
   *   #globalEstop        SVG 링 + 코어 + 힌트 → 처음 누르면 시각이 날아감
   *   #controlRoleButton  #controlRoleText 가 사라져 renderGlobal() 이 그 자리에서
   *                       TypeError → 복구 코드까지 건너뛰어 '반납 중…' 에서 멈춤
   *
   * 그래서 구조가 있으면 disabled + .is-busy 만 걸고, 평문 버튼일 때만 라벨을
   * 바꾼다. 라벨을 꼭 보여줘야 하는 버튼은 전용 처리를 쓴다(setControlBusy).
   */
  function setButtonBusy(button, busy, label) {
    if (!button) return;
    button.disabled = busy;
    button.classList.toggle('is-busy', busy);
    if (button.childElementCount > 0) return;      // 구조가 있는 버튼은 여기까지
    if (busy) {
      if (button.dataset.original === undefined) button.dataset.original = button.textContent;
      button.textContent = label || '처리 중…';
    } else if (button.dataset.original !== undefined) {
      button.textContent = button.dataset.original;
      delete button.dataset.original;
    }
  }

  /**
   * 제어권 버튼 전용 busy. 자식 구조를 건드리지 않고 안쪽 텍스트만 바꾼다.
   * 처리 중에 renderGlobal() 이 돌아도 #controlRoleText 가 살아 있어야 한다.
   */
  function setControlBusy(busy, label) {
    const button = $('#controlRoleButton');
    if (!button) return;
    button.disabled = busy;
    button.classList.toggle('is-busy', busy);
    if (busy) $('#controlRoleText').textContent = label;
    else renderGlobal();
  }

  function addLog(level, node, text) {
    state.logs.push({ time: nowTime(), level, node, text });
    if (state.logs.length > 120) state.logs.shift();
    renderLogs();
  }

  function renderLogs() {
    const filter = $('#logLevel')?.value || '전체';
    const logs = filter === '전체' ? state.logs : state.logs.filter((log) => log.level === filter);
    $('#logWindow').innerHTML = logs.map((log) => `
      <div class="log-line ${esc(log.level.toLowerCase())}">
        <span>${esc(log.time)}</span><b>${esc(log.level)}</b><em>${esc(log.node)}</em><p>${esc(log.text)}</p>
      </div>`).join('');
    $('#logWindow').scrollTop = $('#logWindow').scrollHeight;
  }

  /**
   * 명령 계층(src/commands.js). 로드되기 전이거나 목업 모드면 null 이다.
   * 여기 없는 동안 조작 버튼은 눌러도 아무 일이 없어야지, 화면에서만 성공한
   * 척하면 안 된다 — 그게 이 UI 가 원래 갖고 있던 문제였다.
   */
  const cmd = () => window.ALM_CMD || null;

  /** 명령을 낼 수 있는 상태인지. 못 내면 이유를 말하고 false. */
  function requireCmd() {
    const api = cmd();
    if (!api) {
      toast('명령 계층이 없습니다', '페이지를 새로고침하세요.', 'error');
      return null;
    }
    if (!api.hasToken()) {
      toast('백엔드 토큰이 필요합니다', '설정에서 토큰을 입력하세요.', 'warning');
      openSettings();
      return null;
    }
    return api;
  }

  function canOperate(message = true) {
    if (!state.hasControl) {
      if (message) toast('제어권이 없습니다', '관전 모드에서는 조회만 가능합니다.', 'warning');
      return false;
    }
    if (state.estop) {
      if (message) toast('E-STOP이 활성화되어 있습니다', '안전 확인 후 정지를 해제하세요.', 'error');
      return false;
    }
    return true;
  }

  /**
   * 지금 이 탭으로 갈 수 없는 이유. 갈 수 있으면 빈 문자열.
   *
   * 판정을 함수로 뽑은 이유는 **버튼 비활성과 같은 근거를 써야 하기 때문**이다.
   * 탭은 막았는데 사이드바 버튼은 멀쩡하면 조작자는 무엇이 참인지 알 수 없다.
   */
  function tabLockReason(tab) {
    if (state.systemState === 'MAPPING' && (tab === 'navigation' || tab === 'manual')) {
      return tab === 'navigation'
        ? 'SLAM 매핑 중에는 자율주행을 쓸 수 없습니다 — FAST-LIO 가 두 개 뜹니다.'
        : '매핑 중 차량 이동은 외부 조이스틱으로만 수행합니다.';
    }
    // 주행 중 수동주행 탭은 막는다. 여기서 동작권을 가져가면 Nav2 가 목표를
    // 쥔 채로 twist 만 끊기는 상태가 되고, 화면 어디에도 그게 안 보인다.
    if (tab === 'manual' && state.nav?.state === 'RUNNING') {
      return '자율주행 중입니다 — 수동으로 바꾸려면 먼저 주행을 중단하세요.';
    }
    return '';
  }

  function switchTab(tab) {
    if (!TAB_META[tab]) return;
    // ⚠ 예전에는 토스트만 띄우고 **그대로 전환했다.** '잠겨 있습니다' 라고
    //   말해놓고 안 잠근 것이라, 조작자는 잠긴 줄 알면서 그 탭에서 버튼을
    //   누를 수 있었다. 명령 경로가 실경로가 된 뒤로는 그게 그냥 위험하다.
    //   막을 거면 막고, 못 막을 거면 말을 하지 말아야 한다.
    const blocked = tabLockReason(tab);
    if (blocked) {
      toast('탭이 잠겨 있습니다', blocked, 'warning');
      return;
    }
    state.tab = tab;
    $$('.tab-panel').forEach((panel) => panel.classList.toggle('active', panel.id === `tab-${tab}`));
    $$('.nav-item').forEach((item) => item.classList.toggle('active', item.dataset.tab === tab));
    $('#pageEyebrow').textContent = TAB_META[tab][0];
    $('#pageTitle').textContent = TAB_META[tab][1];
    if (tab !== 'manual') stopManualCommand();
  }

  function renderGlobal() {
    $('#activeMapText').textContent = state.activeMap;
    $('#mappingActiveMapText').textContent = state.activeMap;
    $('#settingsMapSelect').value = state.activeMap;
    $('#profileDesc').textContent = PIPELINE_DESC;
    $('#newMapButton').disabled = state.systemState !== 'IDLE';
    $('#batteryTop').textContent = `${Math.round(state.metrics.battery)}%`;
    $('#batteryFill').style.width = `${state.metrics.battery}%`;
    $('#controlRoleText').textContent = state.hasControl ? '내가 보유' : '관전 모드';
    $('#controlRoleButton').classList.toggle('owned', state.hasControl);
    $('#controlRoleButton').classList.toggle('spectator', !state.hasControl);
    $('#manualControlCheck').textContent = state.hasControl ? '보유' : '없음';

    $('#safetyStrip').classList.toggle('estopped', state.estop);
    $('#releaseEstop').classList.toggle('hidden', !state.estop);
    $('#globalEstop').disabled = state.estop;
    $('#safetyTitle').textContent = state.estop ? 'E-STOP 활성화' : '안전 계통 정상';
    $('#safetySub').textContent = state.estop
      ? '속도 명령 차단 · Nav2 목표 취소 · MCU 정지 지령 유지'
      : 'E-STOP 해제 · 명령 타임아웃 감시 중 · 활성 fault 없음';
    $('#monEstop').textContent = state.estop ? '활성' : '해제';
    $('#monEstop').className = state.estop ? 'text-danger' : 'text-ok';

    $$('.nav-item').forEach((item) => {
      const locked = state.systemState === 'MAPPING' && ['navigation', 'manual'].includes(item.dataset.tab);
      item.classList.toggle('locked', locked);
      item.title = locked ? '매핑 중 조작 잠금' : '';
    });
  }

  /**
   * E-STOP. 제어권(락)이 없어도 누를 수 있다 — 서버도 이 엔드포인트만 락에서
   * 빼 두었다. 남이 조작 중이라고 로봇을 못 세우면 그게 사고다.
   *
   * state.estop 을 여기서 true 로 만들지 않는다. 실제 정지 여부는
   * /mcu/state + /mcu/command 를 보고 ingest.js 가 정한다. 낙관적으로 먼저
   * '정지됨'을 그리면, 명령이 안 나갔는데 멈춘 줄 아는 상황이 만들어진다.
   */
  async function triggerEstop() {
    const api = requireCmd();
    if (!api) return;
    stopManualCommand();
    const button = $('#globalEstop');
    setButtonBusy(button, true, '정지 요청 중…');
    try {
      await api.estop();
      addLog('ERROR', 'alm_web_backend', 'E-STOP 요청을 발행했습니다.');
      toast('비상정지를 요청했습니다', 'MCU 상태로 반영을 확인하세요.', 'error');
    } catch {
      /* commands.js 가 이미 토스트를 띄웠다 */
    } finally {
      setButtonBusy(button, false);
    }
  }

  function requestEstopRelease() {
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">SAFETY CONFIRMATION</p><h2>E-STOP 해제</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">차량 주변이 안전하고 MCU fault가 없으며 모든 조작 장치가 중립인지 확인하세요. 아래 문구를 입력해야 해제할 수 있습니다.</p>
      <label class="modal-field"><span>확인 문구</span><input id="releasePhrase" autocomplete="off" placeholder="정지 해제" /></label></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmRelease">해제 확인</button></div>`);
    $('#confirmRelease').addEventListener('click', async (event) => {
      if ($('#releasePhrase').value.trim() !== '정지 해제') {
        toast('확인 문구가 일치하지 않습니다', '“정지 해제”를 정확히 입력하세요.', 'warning');
        return;
      }
      const api = requireCmd();
      if (!api) return;
      setButtonBusy(event.currentTarget, true, '해제 중…');
      try {
        // 로봇 쪽 래치를 푸는 것은 command_manager 의 서비스뿐이다.
        // 여기서 state.estop 을 내리지 않는다 — /mcu/command 로 실제 해제가
        // 확인되면 ingest.js 가 내린다. 거부되면(MCU fault 유지 등) 409 다.
        await api.releaseEstop('web');
        addLog('INFO', 'command_manager', 'E-STOP 래치 해제 요청이 승인되었습니다.');
        closeModal();
        toast('E-STOP 해제를 요청했습니다', '주행 전 주변 안전을 다시 확인하세요.', 'success');
      } catch {
        /* 거부 사유는 commands.js 가 그대로 띄운다 */
      } finally {
        setButtonBusy(event.currentTarget, false);
      }
    });
  }

  /**
   * 웹 세션 제어권 — 접속한 브라우저 여럿 중 누가 조작하는가.
   *
   * ⚠ cmd_arbiter 의 동작권(auto/teleop)과는 **다른 축**이다. 그쪽은 로봇이
   * 자율을 따르는지 텔레옵을 따르는지고, 이쪽은 웹 클라이언트 사이의 문제다.
   * 둘을 섞으면 다중 접속에서 사고가 난다.
   *
   * 판정은 서버가 한다. state.hasControl 은 서버 응답의 캐시일 뿐이다.
   */
  async function toggleControl() {
    const api = requireCmd();
    if (!api) return;
    if (api.hasControl) {
      if (state.manual.enabled || state.nav.state === 'RUNNING') {
        toast('제어권을 반납할 수 없습니다', '진행 중인 주행을 먼저 종료하세요.', 'warning');
        return;
      }
      setControlBusy(true, '반납 중…');
      try {
        await api.releaseControl();
        toast('제어권을 반납했습니다', '현재 관전 모드입니다.');
      } finally {
        // finally 로 감싸는 이유: 여기서 무슨 일이 나든 버튼은 반드시 돌아와야
        // 한다. 안 그러면 '반납 중…' 에서 멈춰 다시 누를 수조차 없다.
        setControlBusy(false);
      }
      return;
    }
    setControlBusy(true, '확보 중…');
    try {
      await api.acquireControl(location.hostname);
      toast('제어권을 확보했습니다', '조작 기능이 활성화되었습니다.', 'success');
    } catch {
      /* 409(다른 접속자 보유)는 commands.js 가 안내한다 */
    } finally {
      setControlBusy(false);
    }
  }

  function renderMappingSteps() {
    const icons = { pending: '○', running: '●', done: '✓', failed: '!', skipped: '−' };
    const labels = { pending: '대기', running: '진행 중', done: '완료', failed: '실패', skipped: '건너뜀' };
    $('#mappingStepper').innerHTML = state.mappingSteps.map((step, index) => `
      <li class="${esc(step.status)}"><span class="step-index">${icons[step.status]}</span><div><strong>${index + 1}. ${esc(step.title)}</strong><small>${esc(step.detail)}</small></div><b>${labels[step.status]}</b></li>`).join('');
    syncSaveHint();
  }

  /**
   * 매핑 중에는 '3D 맵 저장' 이 다음에 할 일이다. 그 칸을 빛나게 해서 눈에
   * 띄게 한다. 오른쪽 후처리 패널은 카드가 셋 다 똑같이 생겨서, 지금 눌러야
   * 할 것이 무엇인지 화면이 말해주지 않았다.
   */
  function syncSaveHint() {
    const button = $('#savePcd');
    const row = button?.closest('.asset-row');
    if (!row || !button) return;
    const mapping = state.systemState === 'MAPPING';
    row.classList.toggle('is-next', mapping);
    // 누르면 SLAM 도 같이 끝난다. 라벨이 '저장' 이면 그 사실을 숨기는 셈이다.
    // (busy 중에는 setButtonBusy 가 라벨을 들고 있으므로 건드리지 않는다)
    if (button.dataset.original === undefined) {
      button.textContent = mapping ? '저장 후 종료' : '저장';
    }
  }

  /**
   * SLAM 매핑 시작 — alm_navigation/slam.launch.py 를 로봇에서 띄운다.
   *
   * 예전 목업은 setTimeout 으로 단계가 저절로 진행되는 척했다. 이제는 서버가
   * 프로세스를 실제로 띄우고, 단계 상태는 **실제 자산**(map_manager 가 보는
   * maps/ 의 파일)을 따라간다. 경과 시간도 지어내지 않고 실제 기동 시각부터 센다.
   */
  async function startMapping() {
    if (!canOperate()) return;
    if (state.systemState !== 'IDLE') {
      toast('현재 시작할 수 없습니다', `시스템 상태: ${state.systemState}`, 'warning');
      return;
    }
    const api = requireCmd();
    if (!api) return;

    const target = state.activeMap;
    const entry = state.maps.find((map) => map.name === target);
    // 매핑을 시작하면 서버가 이 맵의 산출물을 **전부** 지운다 (cloud/grid/DB).
    // 하나라도 있으면 확인을 받는다 — cloud.pcd 만 보고 판단하면, 2D 나 DB 만
    // 남아 있는 맵에서 조용히 지워진다.
    const existing = (entry?.assets ?? []).filter((asset) => asset.present);
    if (existing.length && !await confirmOverwrite(target, existing)) return;

    const button = $('#startMapping');
    setButtonBusy(button, true, '기동 중…');
    try {
      const result = await api.startMapping(target, existing.length > 0);
      state.systemState = 'MAPPING';
      state.mappingSaved = false;
      state.mappingElapsed = 0;
      state.mappingStartedAt = Date.now();
      // 이전 세션의 누적 점군을 지운다. 안 지우면 새 맵 위에 옛 맵이 겹쳐
      // 보여서, 화면상으로는 이미 다 매핑된 것처럼 보인다.
      window.ALM_RENDERER3D?.resetAccumulation();
      $('#stopMapping').disabled = false;
      $('#mappingStateLabel').textContent = '매핑 진행 중';
      // 센서·엔진 기동은 launch 가 한 번에 하므로 둘을 한꺼번에 done 으로 둔다.
      // 이후 단계는 자산이 실제로 생기면 ingest.js 가 done 으로 접는다.
      markStep('sensor', 'done');
      markStep('engine', 'done');
      markStep('scanning', 'running');
      if (result.cleared?.length) {
        addLog('WARN', 'alm_web_backend', `기존 자산 삭제: ${result.cleared.join(', ')}`);
      }
      addLog('INFO', 'alm_web_backend',
        `slam.launch.py 기동 (pid=${result.process?.pid}) → ${result.target}`);
      // 자산을 지웠으므로 저장된 맵 레이어도 비운다. map_manager 가 5초 안에
      // 알려주긴 하지만, 그 사이 화면에 방금 지운 맵이 남아 있으면 헷갈린다.
      window.ALM_RENDERER3D?.onPriorCloud(null);
      state.mappingTimer = setInterval(() => {
        state.mappingElapsed = Math.round((Date.now() - state.mappingStartedAt) / 1000);
        $('#mappingElapsed').textContent = fmtDuration(state.mappingElapsed);
      }, 1000);
      toast('SLAM 매핑을 시작했습니다', '차량 이동은 외부 조이스틱으로 수행하세요.', 'success');
    } catch {
      state.systemState = 'IDLE';
    } finally {
      setButtonBusy(button, false);
      $('#startMapping').disabled = state.systemState === 'MAPPING';
      renderMappingSteps(); renderGlobal();
    }
  }

  function markStep(key, status) {
    const step = state.mappingSteps.find((item) => item.key === key);
    if (step) step.status = status;
  }

  /**
   * SLAM 이 끝났다. 화면에 쌓아둔 누적 점군을 버린다.
   *
   * 저장(/map_save)하지 않았다면 그 점군은 어디에도 남지 않는다. 화면에만
   * 계속 띄워두면 "맵이 있다"고 착각하게 만든다 — 정확히 이 UI 가 원래 갖고
   * 있던 문제다. 저장했다면 cloud.pcd 가 생겼고 prior_cloud_publisher 가
   * 그걸 저장된 맵 레이어로 다시 보내주므로 잃는 것이 없다.
   */
  function discardAccumulation() {
    window.ALM_RENDERER3D?.resetAccumulation();
  }

  /**
   * 서버가 본 slam 프로세스 상태가 바뀌었다 (main.js 의 health 폴링).
   *
   * 이 탭에서 시작한 매핑이면 systemState 가 이미 MAPPING 이라 할 일이 없다.
   * 문제는 **다른 경로로 시작된 경우**다 — CLI, 다른 브라우저, 또는 이 탭을
   * 열기 전에 이미 돌고 있던 경우. 그때 화면이 'IDLE' 이라고 말하면 조작자는
   * SLAM 시작 버튼을 다시 누르게 되고, 서버는 409 로 막지만 화면은 계속
   * 거짓말을 하고 있는 상태가 된다.
   */
  function onSlamRunningChange(running, slot) {
    const label = $('#mappingStateLabel');
    if (running && state.systemState !== 'MAPPING') {
      state.systemState = 'MAPPING';
      state.mappingStartedAt = Date.now() - Math.round((slot?.uptime_sec || 0) * 1000);
      markStep('sensor', 'done'); markStep('engine', 'done'); markStep('scanning', 'running');
      if (label) label.textContent = '매핑 진행 중 (다른 경로에서 시작됨)';
      $('#startMapping').disabled = true;
      $('#stopMapping').disabled = false;
      clearInterval(state.mappingTimer);
      state.mappingTimer = setInterval(() => {
        state.mappingElapsed = Math.round((Date.now() - state.mappingStartedAt) / 1000);
        $('#mappingElapsed').textContent = fmtDuration(state.mappingElapsed);
      }, 1000);
      addLog('WARN', 'alm_web_backend',
        `이 탭 밖에서 시작된 매핑을 감지했습니다 (pid=${slot?.pid}).`);
    } else if (!running && state.systemState === 'MAPPING') {
      clearInterval(state.mappingTimer);
      state.mappingTimer = null;
      state.systemState = 'IDLE';
      markStep('scanning', 'done');
      discardAccumulation();
      if (label) label.textContent = state.mappingSaved ? '매핑 완료' : '매핑 종료됨';
      $('#startMapping').disabled = false;
      $('#stopMapping').disabled = true;
      addLog('INFO', 'alm_web_backend', '매핑 프로세스가 종료되었습니다.');
    }
    renderMappingSteps(); renderGlobal(); renderMapOptions();
  }

  const ASSET_FILES = {
    cloud: 'cloud.pcd',
    grid: 'grid.pgm · grid.yaml',
    fpfh: 'fpfh_map.meta · fpfh_map_*.pcd',
  };

  /**
   * 재매핑 전 확인. 서버는 이 맵의 산출물을 **전부 지우고** 시작한다.
   *
   * cloud.pcd 만 덮는 게 아닌 이유: 그러면 grid.pgm 과 fpfh_map* 이 옛 점군에서
   * 만들어진 채 남아, 짝이 안 맞는 자산이 한 폴더에 섞인다. 그 DB 로 측위를
   * 돌리면 엉뚱한 곳에 수렴한다.
   */
  /**
   * 되돌릴 수 없는 조작 앞의 일반 확인창. body 는 신뢰된 문자열이므로 HTML 을
   * 그대로 쓴다 (호출부는 전부 이 파일 안의 고정 문구다 — 사용자 입력을
   * 여기에 넣지 말 것).
   */
  function confirmModal({ title, body, confirm = '계속', kicker = 'CONFIRM' }) {
    return new Promise((resolve) => {
      openModal(`
        <div class="modal-head"><div><p class="section-kicker">${esc(kicker)}</p><h2>${esc(title)}</h2></div><button class="close-button" data-close-modal>×</button></div>
        <div class="modal-body"><p class="modal-copy">${body}</p></div>
        <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmModalOk">${esc(confirm)}</button></div>`);
      let settled = false;
      const finish = (value) => { if (!settled) { settled = true; resolve(value); } };
      $('#confirmModalOk').addEventListener('click', () => { closeModal(); finish(true); });
      $$('[data-close-modal]', $('#modal')).forEach((node) =>
        node.addEventListener('click', () => finish(false)));
    });
  }

  function confirmOverwrite(mapName, existing) {
    const list = existing.map((asset) =>
      `<li><b>${esc(ASSET_LABEL[asset.kind] || asset.kind)}</b> — <code>${esc(ASSET_FILES[asset.kind] || asset.kind)}</code>`
      + `${asset.detail ? ` <small>${esc(asset.detail)}</small>` : ''}</li>`).join('');
    return new Promise((resolve) => {
      openModal(`
        <div class="modal-head"><div><p class="section-kicker">DESTRUCTIVE</p><h2>기존 자산을 모두 지울까요?</h2></div><button class="close-button" data-close-modal>×</button></div>
        <div class="modal-body"><p class="modal-copy"><b>${esc(mapName)}</b> 로 매핑을 시작하면 아래 파일을 <b class="text-danger">전부 삭제</b>하고 처음부터 만듭니다. 되돌릴 수 없습니다.</p>
        <ul class="wipe-list">${list}</ul>
        <p class="modal-copy">기존 맵을 남기려면 취소하고 <b>＋ 새 맵 만들기</b>로 다른 폴더를 만드세요.</p></div>
        <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="danger-text-button" id="confirmOverwrite">지우고 새로 매핑</button></div>`);
      let settled = false;
      const finish = (value) => { if (!settled) { settled = true; resolve(value); } };
      $('#confirmOverwrite').addEventListener('click', () => { closeModal(); finish(true); });
      $$('[data-close-modal]', $('#modal')).forEach((node) =>
        node.addEventListener('click', () => finish(false)));
    });
  }

  /**
   * SLAM 프로세스를 실제로 내린다. 성공하면 true.
   *
   * 종료 버튼과 저장 버튼이 둘 다 이걸 부른다 (저장이 곧 매핑의 끝이므로).
   * 어느 버튼이 눌렸는지에 따라 busy 표시가 달라지므로 버튼을 받는다.
   */
  async function stopSlamProcess(button) {
    const api = requireCmd();
    if (!api) return false;
    setButtonBusy(button, true, '종료 중…');
    try {
      // 서버는 프로세스 그룹에 SIGINT → SIGTERM → SIGKILL 순으로 보낸다.
      // launch 가 자식 노드를 역순으로 정리할 시간을 주므로 최대 15초 걸린다.
      await api.stopMapping();
      return true;
    } catch {
      return false;
    } finally {
      setButtonBusy(button, false);
    }
  }

  /** 종료 후 화면 상태를 맞춘다 (프로세스를 내린 뒤에만 부른다). */
  function applyMappingStopped() {
    clearInterval(state.mappingTimer);
    state.mappingTimer = null;
    state.systemState = 'IDLE';
    markStep('scanning', 'done');
    discardAccumulation();
    $('#mappingStateLabel').textContent = state.mappingSaved ? '매핑 완료' : '저장 전 종료';
    $('#startMapping').disabled = false;
    $('#stopMapping').disabled = true;
    addLog('INFO', 'alm_web_backend', 'slam.launch.py 를 역순으로 종료했습니다.');
    renderMappingSteps(); renderGlobal();
  }

  function stopMapping() {
    if (state.systemState !== 'MAPPING') return;
    const proceed = async () => {
      if (!await stopSlamProcess($('#stopMapping'))) return;
      applyMappingStopped();
      closeModal();
      toast('SLAM 프로세스를 종료했습니다', state.mappingSaved ? '저장된 맵을 사용할 수 있습니다.' : 'PCD가 저장되지 않았습니다.', state.mappingSaved ? 'success' : 'warning');
    };
    if (!state.mappingSaved) {
      openModal(`<div class="modal-head"><div><p class="section-kicker">UNSAVED MAP</p><h2>저장하지 않고 종료할까요?</h2></div><button class="close-button" data-close-modal>×</button></div><p class="modal-copy">현재 누적 맵이 PCD로 저장되지 않았습니다. 종료하면 이번 매핑 결과를 잃을 수 있습니다.</p><div class="modal-actions"><button class="secondary-button" data-close-modal>계속 매핑</button><button class="danger-text-button" id="forceStopMapping">저장 없이 종료</button></div>`);
      $('#forceStopMapping').addEventListener('click', proceed);
    } else proceed();
  }

  /**
   * /map_save (std_srvs/Trigger). 저장 경로는 매핑 시작 때 정해진 것을 따른다.
   *
   * ⚠ 누적 점군이 비어 있으면 FAST-LIO 가 pcl 예외를 안 잡고 죽는다(상류 버그).
   * 서버가 그 상황을 감지해 즉시 사유를 알려주므로 여기서는 그대로 보여준다.
   */
  async function savePcd() {
    if (!canOperate()) return;
    if (!['MAPPING', 'IDLE'].includes(state.systemState)) return;
    const api = requireCmd();
    if (!api) return;
    const button = $('#savePcd');
    let saved = false;
    setButtonBusy(button, true, '저장 중…');
    markStep('save_pcd', 'running'); renderMappingSteps();
    addLog('INFO', 'map_save', '/map_save 서비스를 호출합니다 (점군이 크면 수십 초).');
    try {
      const result = await api.saveMap();
      state.mappingSaved = true;
      markStep('save_pcd', 'done');
      addLog('INFO', 'map_save', `저장 완료: ${result.saved_to}`);
      saved = true;
    } catch {
      markStep('save_pcd', 'failed');
    } finally {
      renderMappingSteps();
      setButtonBusy(button, false);
    }
    if (!saved) return;

    // 저장이 곧 매핑의 끝이다. 저장해 놓고 SLAM 을 계속 돌리면 이후 스캔은
    // 어디에도 안 남으면서 CPU 와 메모리만 먹는다. 그래서 같이 내린다.
    //
    // ⚠ 순서가 중요하다. 먼저 내리면 fast_lio 가 죽어서 /map_save 를 받을 노드가
    //   없다. 반드시 저장이 끝난 뒤에 종료한다.
    if (state.systemState !== 'MAPPING') {
      // 자산 카드 문구는 여기서 지어내지 않는다 — map_manager 가 파일 헤더를
      // 읽어 5초 안에 /alm/map_inventory 로 실제 값을 보내준다.
      toast('3D 맵을 저장했습니다', 'map_manager 가 자산 상태를 갱신합니다.', 'success');
      return;
    }
    if (await stopSlamProcess(button)) {
      applyMappingStopped();
      toast('저장하고 매핑을 끝냈습니다', 'SLAM 프로세스를 종료했습니다. 이제 2D 변환과 측위 DB 를 만들 수 있습니다.', 'success');
    } else {
      // 저장은 됐고 종료만 실패한 상태다. 뭉뚱그리면 조작자가 저장까지 실패한
      // 줄 안다 — 다시 저장을 누르게 만든다.
      toast('저장은 됐지만 종료에 실패했습니다', '종료 버튼으로 다시 시도하세요.', 'warning');
    }
  }

  function openPcd2Pgm() {
    // 이번 세션에서 저장했는지(state.mappingSaved)가 아니라, 파일이 실제로
    // 있는지를 본다. 이미 완성된 맵을 다시 열었을 때도 변환할 수 있어야 한다.
    if (!hasCloudAsset()) {
      toast('PCD가 필요합니다', '먼저 3D 맵을 저장하세요.', 'warning');
      return;
    }
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">PCD TO PGM</p><h2>2D 맵 변환</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">높이 밴드를 조정해 장애물 단면을 생성합니다. 작업 진행률은 임의 추정하지 않고 단계 상태로 표시합니다.</p>
      <div class="modal-grid"><label class="modal-field"><span>Resolution</span><input id="pgmResolution" type="number" value="0.05" step="0.01"></label><label class="modal-field"><span>Min points</span><input id="pgmMinPoints" type="number" value="1"></label><label class="modal-field"><span>Z min</span><input id="pgmZMin" type="number" value="-0.3" step="0.1"></label><label class="modal-field"><span>Z max</span><input id="pgmZMax" type="number" value="1.5" step="0.1"></label></div>
      <p class="helper">z 밴드는 라이다 마운트 높이에 따라 달라집니다. 실행 후 출력되는 z 분포를 보고 지면 위 0.2~1.5 m 로 맞추세요.</p>
      <pre class="job-log" id="jobLog">대기 중</pre></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>닫기</button><button class="primary-button" id="runPgmJob">변환 실행</button></div>`);
    $('#runPgmJob').addEventListener('click', (event) => runAssetJob({
      button: event.currentTarget,
      stepKey: 'pcd2pgm',
      label: 'pcd2pgm',
      run: (api) => api.runPcd2Pgm({
        map: state.activeMap,
        resolution: Number($('#pgmResolution').value),
        min_points: Number($('#pgmMinPoints').value),
        z_min: Number($('#pgmZMin').value),
        z_max: Number($('#pgmZMax').value),
      }),
      done: '2D 맵 변환이 완료되었습니다',
    }));
  }

  /**
   * 자산 생성 작업(pcd2pgm / fpfh_map_builder)의 공통 실행기.
   *
   * 진행률을 만들어내지 않는다. 두 스크립트 모두 퍼센트를 내지 않으므로,
   * **실제 stdout 줄**을 그대로 흘린다. 목업 시절의 25%→50%→75% 막대는
   * 아무것도 뜻하지 않는 애니메이션이었다.
   */
  async function runAssetJob({ button, stepKey, label, run, done }) {
    const api = requireCmd();
    if (!api) return;
    const logNode = $('#jobLog');
    if (logNode) logNode.textContent = '';
    setButtonBusy(button, true, '실행 중…');
    markStep(stepKey, 'running'); renderMappingSteps();

    const append = (line) => {
      addLog('INFO', label, line);
      if (!logNode) return;
      logNode.textContent += (logNode.textContent ? '\n' : '') + line;
      logNode.scrollTop = logNode.scrollHeight;
    };

    try {
      const started = await run(api);
      const result = await api.followJob(started.id, { onLine: append });
      if (result.state === 'succeeded') {
        markStep(stepKey, 'done');
        toast(done, 'map_manager 가 자산 상태를 갱신합니다.', 'success');
      } else {
        markStep(stepKey, 'failed');
        toast('작업이 실패했습니다', `${label} · 종료 코드 ${result.returncode}`, 'error');
      }
    } catch (error) {
      markStep(stepKey, 'failed');
      if (error?.status === undefined) toast('작업 실패', String(error), 'error');
    } finally {
      renderMappingSteps();
      setButtonBusy(button, false);
    }
  }

  /**
   * FPFH 측위 DB 생성.
   *
   * voxel 이 이 작업의 핵심 손잡이다. 맵을 그 격자로 줄인 뒤 남은 점마다
   * 기술자를 만들므로, **voxel 이 곧 DB 의 feature 개수**를 정한다. 적으면
   * 정합이 자주 거절되고([MATCH] 문턱 20개), 너무 많으면 Orin Nano 에서
   * 초기화가 느려진다.
   *
   * 여기서 고른 값은 측위 기동 시 fpfh_map.meta 에서 다시 읽혀 localizer 로
   * 넘어간다 — 화면에서 바꿨는데 localizer 만 옛 값으로 도는 일은 없다.
   */
  function openFpfhBuilder() {
    if (!hasCloudAsset()) {
      toast('PCD가 필요합니다', '먼저 3D 맵을 저장하세요.', 'warning');
      return;
    }
    const entry = state.maps.find((map) => map.name === state.activeMap);
    const cloud = entry?.assets?.find((asset) => asset.kind === 'cloud');
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">FPFH DATABASE</p><h2>측위 DB 생성</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy"><b>${esc(state.activeMap)}</b> 의 <code>cloud.pcd</code>${cloud?.detail ? ` <small>(${esc(cloud.detail)})</small>` : ''} 를 격자로 줄인 뒤, 남은 점마다 주변 형상을 33차원 지문으로 만들어 저장합니다. 측위는 이 지문으로 현재 스캔의 위치를 맵 전체에서 찾습니다.</p>
      <div class="modal-grid">
        <label class="modal-field"><span>Voxel (m)</span><input id="fpfhVoxel" type="number" value="0.5" step="0.05" min="0.05" max="5"></label>
        <label class="modal-field"><span>Normal radius (m)</span><input id="fpfhNormalRadius" type="number" value="1.0" step="0.1" min="0.1" max="10"></label>
        <label class="modal-field"><span>Feature radius (m)</span><input id="fpfhFeatureRadius" type="number" value="2.5" step="0.1" min="0.1" max="20"></label>
        <label class="modal-field"><span>Z min (m)</span><input id="fpfhZMin" type="number" value="-0.35" step="0.05"></label>
        <label class="modal-field"><span>Z max (m)</span><input id="fpfhZMax" type="number" value="1.0" step="0.05"></label>
      </div>
      <p class="helper" id="fpfhHint"></p>
      <p class="helper"><b>voxel 을 줄이면</b> 남는 점이 많아져 feature 가 늘고 정합이 잘 붙습니다. 대신 생성 시간·파일 크기가 커지고, 매 스캔마다 FPFH 를 다시 계산하는 측위 쪽도 무거워집니다. 실내 복도처럼 형상이 반복되는 곳은 0.25~0.3 이 무난하고, 넓고 특징이 뚜렷한 곳은 0.5 로 충분합니다.<br>
      <b>반경 두 개는 voxel 과 같이 움직여야 합니다.</b> normal ≈ voxel×2, feature ≈ voxel×5 가 기준입니다. voxel 만 줄이면 반경 안에 점이 너무 많이 들어와 지문이 뭉개집니다.</p>
      <pre class="job-log" id="jobLog">대기 중</pre></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>닫기</button><button class="primary-button" id="runFpfhJob">DB 생성</button></div>`);

    // voxel 을 만지면 두 반경을 같이 끌어준다. 사용자가 직접 고친 뒤에는
    // 건드리지 않는다 — 손으로 넣은 값을 화면이 되돌리면 안 된다.
    const voxel = $('#fpfhVoxel');
    const normal = $('#fpfhNormalRadius');
    const feature = $('#fpfhFeatureRadius');
    let radiiTouched = false;
    for (const node of [normal, feature]) {
      node.addEventListener('input', () => { radiiTouched = true; });
    }
    const syncHint = () => {
      const v = Number(voxel.value) || 0.5;
      if (!radiiTouched) {
        normal.value = (v * 2).toFixed(2);
        feature.value = (v * 5).toFixed(2);
      }
      // 점 개수는 형상에 따라 다르므로 예측하지 않는다. 대신 지금 DB 가 있으면
      // 그 실측치를 기준선으로 보여준다 — 비교할 대상이 있어야 판단이 된다.
      const fpfh = entry?.assets?.find((asset) => asset.kind === 'fpfh');
      $('#fpfhHint').innerHTML = fpfh?.present
        ? `현재 DB: <b>${esc(fpfh.detail || '?')}</b> — 정합이 <code>[MATCH]</code> 에서 자주 거절되면 voxel 을 줄여 다시 만드세요.`
        : '아직 DB 가 없습니다. 먼저 기본값으로 만든 뒤, 정합이 잘 안 붙으면 voxel 을 줄여 다시 만드는 순서를 권합니다.';
    };
    voxel.addEventListener('input', syncHint);
    syncHint();

    $('#runFpfhJob').addEventListener('click', (event) => runAssetJob({
      button: event.currentTarget,
      stepKey: 'fpfh_db',
      label: 'fpfh_map_builder',
      run: (api) => api.runFpfh({
        map: state.activeMap,
        voxel: Number(voxel.value),
        normal_radius: Number(normal.value),
        feature_radius: Number(feature.value),
        z_min: Number($('#fpfhZMin').value),
        z_max: Number($('#fpfhZMax').value),
      }),
      done: 'FPFH 측위 DB를 생성했습니다',
    }));
  }

  /** 활성 맵에 cloud.pcd 가 실제로 있는가 (map_manager 가 본 사실 기준). */
  function hasCloudAsset() {
    const entry = state.maps.find((map) => map.name === state.activeMap);
    return Boolean(entry?.assets?.find((a) => a.kind === 'cloud' && a.present));
  }

  function openMapManager() {
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">MAP ASSETS</p><h2>맵 자산 관리</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div id="mapManagerBody">${mapManagerBody()}</div>
      <p class="modal-copy">각 자산은 부모 <b>cloud.pcd</b>와 짝이 맞을 때만 사용할 수 있습니다. 2D 맵은 <code>pcd2pgm</code>, 측위 DB는 <code>fpfh_map_builder</code>가 만듭니다.<br>
      <b>활성</b>으로 지정하면 로봇의 <code>maps/active.yaml</code>이 바뀝니다. 이미 실행 중인 노드에는 다음 기동부터 반영됩니다.</p>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>닫기</button><button class="primary-button" id="openNewMapForm">＋ 새 맵 만들기</button></div>`);
    $('#openNewMapForm').addEventListener('click', openNewMapModal);

    // 위임 바인딩 — 목록은 인벤토리가 올 때마다 통째로 다시 그려지므로
    // 버튼마다 붙이면 갱신 때마다 다시 붙여야 한다.
    $('#mapManagerBody').addEventListener('click', async (event) => {
      const button = event.target.closest('[data-activate]');
      if (!button) return;
      setButtonBusy(button, true, '전환 중…');
      try {
        await changeActiveMap(button.dataset.activate);
      } finally {
        setButtonBusy(button, false);
        refreshMapManager();
      }
    });
  }

  /** 모달이 열려 있으면 목록을 최신 인벤토리로 다시 그린다. */
  function refreshMapManager() {
    const node = $('#mapManagerBody');
    if (node) node.innerHTML = mapManagerBody();
  }

  function mapManagerBody() {
    // 받기 전(null)과 진짜로 맵이 없는 것(빈 배열)은 다른 상태다 — 섞지 않는다.
    if (state.mapInventory === null) {
      return '<p class="modal-copy">맵 목록을 읽는 중입니다… (map_manager 미연결)</p>';
    }
    if (state.maps.length === 0) {
      return '<p class="modal-copy">maps/ 에 맵이 없습니다. SLAM으로 첫 맵을 만드세요.</p>';
    }
    return `<div class="map-manager-list">${state.maps.map(mapManagerRow).join('')}</div>`;
  }

  const ASSET_LABEL = { cloud: '3D', grid: '2D', fpfh: 'FPFH' };

  /** 맵 한 줄. 자산 뱃지는 실제 파일 상태 그대로 — present/stale 을 구분해 보여준다. */
  function mapManagerRow(map) {
    const badges = (map.assets || []).map((asset) => {
      const label = ASSET_LABEL[asset.kind] || asset.kind;
      if (!asset.present) return `<b class="cap missing" title="없음">${esc(label)}</b>`;
      if (asset.stale) return `<b class="cap stale" title="${esc(asset.issue)}">${esc(label)} ⚠</b>`;
      return `<b class="cap" title="${esc(asset.detail)}">${esc(label)}</b>`;
    }).join('');
    const status = map.complete
      ? '<span class="map-status ok">완성</span>'
      : '<span class="map-status pending">미완성</span>';
    const active = map.active ? '<span class="map-status active">활성</span>' : '';
    const subtitle = [map.created, map.notes].filter(Boolean).join(' · ') || map.path;

    // 매핑 중에는 못 바꾼다 — 어느 폴더에 쌓이는지 헷갈리게 만들지 않는다.
    // 이 탭에서 시작했는지(systemState)가 아니라 **서버가 보는 프로세스**를
    // 기준으로 판단한다. CLI 나 다른 브라우저가 띄웠어도 매핑은 매핑이다.
    const mapping = state.systemState === 'MAPPING' || state.slamRunning === true;
    const action = map.active
      ? '<button class="ghost-button" disabled>활성</button>'
      : `<button class="ghost-button" data-activate="${esc(map.name)}"${mapping ? ' disabled' : ''}
           title="${mapping ? '매핑 중에는 바꿀 수 없습니다' : 'maps/active.yaml 을 이 맵으로 바꿉니다'}">활성으로</button>`;

    return `<div class="map-manager-item${map.active ? ' is-active' : ''}"><div>
      <strong>${esc(map.label || map.name)} ${status}${active}</strong>
      <small>${esc(map.name)} — ${esc(subtitle)}</small>
      <span>${badges}</span></div>${action}</div>`;
  }

  function renderMapOptions() {
    const select = $('#settingsMapSelect');
    const names = state.maps.length ? state.maps : [{ name: state.activeMap }];
    select.innerHTML = names.map((map) => `<option value="${esc(map.name)}">${esc(map.name)}</option>`).join('');
    select.value = state.activeMap;
    // 매핑 중에는 활성 맵을 바꾸지 못하게 한다 — 저장 대상이 도중에 바뀌면
    // 어느 폴더에 쌓이는지가 모호해진다.
    const busy = state.systemState === 'MAPPING';
    select.disabled = busy || !state.maps.length;
    select.title = busy
      ? '매핑 중에는 활성 맵을 바꿀 수 없습니다'
      : '로봇의 maps/active.yaml 을 바꿉니다';
    // 맵 관리 모달이 열려 있으면 같이 갱신한다. 새 맵을 만들거나 자산이
    // 생겼을 때 모달을 닫았다 다시 열게 만들지 않기 위함이다.
    refreshMapManager();
  }

  /** 활성 맵 전환. 이미 떠 있는 launch 에는 반영되지 않는다(다음 기동부터). */
  async function changeActiveMap(name) {
    const api = requireCmd();
    if (!api) { renderMapOptions(); return false; }
    try {
      const result = await api.setActiveMap(name);
      // ⚠ 누적 점군을 반드시 지운다. 안 지우면 이전 맵에서 쌓인 점군이 남은 채로
      // 새 맵 이름이 표시되어, 화면이 "이 맵은 이렇게 생겼다"고 거짓말한다.
      // (실제로 집에서 매핑한 점군이 alm_lab 을 고른 뒤에도 그대로 떠 있었다)
      window.ALM_RENDERER3D?.resetAccumulation();
      addLog('INFO', 'alm_web_backend', `활성 맵 → ${name}`);
      // 저장 목적지가 어디로 옮겨갔는지 로그에 남긴다. SLAM 실행 중이라 안
      // 옮겨간 경우도 있으므로, 지어내지 말고 서버가 준 값을 그대로 쓴다.
      if (result?.mapping_target) {
        addLog('INFO', 'alm_web_backend', `매핑 저장 위치 → ${result.mapping_target}`);
      }
      toast('활성 맵을 바꿨습니다',
        result?.note || '이미 실행 중인 노드에는 다음 기동부터 반영됩니다.', 'success');
      return true;
    } catch {
      renderMapOptions();   // 실패했으면 서버 값으로 되돌린다
      return false;
    }
  }

  function openNewMapModal() {
    if (state.systemState !== 'IDLE') {
      toast('새 맵을 만들 수 없습니다', '현재 실행 중인 워크플로를 먼저 종료하세요.', 'warning');
      return;
    }
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">NEW MAP</p><h2>새 맵 만들기</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">로봇에 <code>maps/&lt;이름&gt;/</code> 폴더와 <code>manifest.yaml</code>을 만들고, SLAM으로 그 안에 <code>cloud.pcd</code>를 채웁니다.</p>
      <label class="modal-field"><span>맵 이름</span><input id="newMapName" autocomplete="off" placeholder="예: warehouse_b" /></label>
      <label class="modal-field"><span>표시 이름 (선택)</span><input id="newMapLabel" autocomplete="off" placeholder="예: B동 창고 1층" /></label>
      <p class="modal-copy text-danger hidden" id="newMapError"></p></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmNewMap">만들기</button></div>`);
    $('#newMapName').focus();
    $('#confirmNewMap').addEventListener('click', async (event) => {
      const raw = $('#newMapName').value.trim();
      const errorEl = $('#newMapError');
      const fail = (text) => {
        errorEl.textContent = text;
        errorEl.classList.remove('hidden');
      };
      // 서버도 같은 규칙으로 다시 검사한다. 여기 검사는 왕복을 아끼는 용도지
      // 신뢰 경계가 아니다.
      if (!/^[a-zA-Z0-9][a-zA-Z0-9_-]{1,31}$/.test(raw)) {
        fail('영문·숫자로 시작하고 영문·숫자·_·- 조합 2~32자로 입력하세요.');
        return;
      }
      if (state.maps.some((map) => map.name.toLowerCase() === raw.toLowerCase())) {
        fail('이미 존재하는 맵 이름입니다.');
        return;
      }
      const api = requireCmd();
      if (!api) return;
      setButtonBusy(event.currentTarget, true, '만드는 중…');
      try {
        await api.createMap(raw, $('#newMapLabel').value.trim(), '');
        // state.maps 를 여기서 건드리지 않는다. 목록은 오직 map_manager 가
        // 파일시스템에서 본 것만 담는다 — 밀어 넣으면 화면이 다시 거짓말한다.
        closeModal();
        toast('맵 폴더를 만들었습니다', `${raw} · 목록은 곧 갱신됩니다`, 'success');
      } catch (error) {
        fail(error.message);
      } finally {
        setButtonBusy(event.currentTarget, false);
      }
    });
  }

  function svgPoint(event) {
    const svg = $('#navigationMap');
    const point = svg.createSVGPoint();
    point.x = event.clientX; point.y = event.clientY;
    return point.matrixTransform(svg.getScreenCTM().inverse());
  }

  function pixelToMap(x, y) {
    // 라이브에서는 map2d.js 가 실제 OccupancyGrid 의 resolution/origin 으로
    // 계산한 변환을 넣어준다. 목업 좌표계(900×620 뷰박스, 0.02 m/px)는
    // 지도가 하드코딩 도형일 때만 의미가 있다.
    const t = window.ALM_MAP_TRANSFORM;
    if (t) {
      return { x: (x - t.offsetX) / t.scale, y: (t.offsetY - y) / t.scale };
    }
    return { x: (x - 450) * 0.02, y: (310 - y) * 0.02 };
  }

  /** 지도 좌표(m) → SVG 좌표. pixelToMap 의 역변환. */
  function mapToSvg(x, y) {
    const t = window.ALM_MAP_TRANSFORM;
    if (t) return { x: t.offsetX + x * t.scale, y: t.offsetY - y * t.scale };
    return { x: 450 + x / 0.02, y: 310 - y / 0.02 };
  }

  // 이보다 짧게 끌면 '방향 지정 없음'으로 본다 [SVG 단위].
  // 손이 떨려서 2~3 단위 움직이는 것을 헤딩으로 읽으면, 클릭할 때마다
  // 무작위 방향이 박힌다 — yaw_goal_tolerance 가 11.5° 라 그건 그대로 제약이 된다.
  const GOAL_DRAG_MIN = 8;

  function renderWaypoints() {
    $('#waypointCount').textContent = state.waypoints.length;
    $('#waypointEmpty').classList.toggle('hidden', state.waypoints.length > 0);
    $('#waypointList').innerHTML = state.waypoints.map((point, index) => `
      <div class="waypoint-item" draggable="true" data-id="${esc(point.id)}"><span class="waypoint-number">${index + 1}</span><div><strong>${esc(point.label)}</strong><small>x ${point.x.toFixed(2)} · y ${point.y.toFixed(2)} · yaw ${(Number(point.yaw) || 0).toFixed(0)}°</small></div><button data-remove-waypoint="${esc(point.id)}">×</button></div>`).join('');
    $$('[data-remove-waypoint]').forEach((button) => button.addEventListener('click', () => {
      state.waypoints = state.waypoints.filter((point) => String(point.id) !== button.dataset.removeWaypoint);
      renderWaypoints();
    }));

    // 위치는 **매번 지도 좌표에서 다시 계산한다.** 예전에는 클릭 순간의 SVG
    // 좌표(px/py)를 박아 뒀는데, 맵을 바꾸면 축척이 달라져 핀이 엉뚱한 곳에
    // 남았다. 진실은 미터 좌표이고 화면 좌표는 그때그때 파생값이다.
    //
    // 핀 자체는 로봇 마커와 달리 **축척을 따라가지 않는다.** 핀은 물리적
    // 물체가 아니라 조작용 표식이라, 넓은 맵에서 몇 픽셀로 줄어들면 누를 수가
    // 없다. 지도 앱의 핀이 확대해도 같은 크기인 것과 같은 이유다.
    $('#waypointLayer').innerHTML = state.waypoints.map((point, index) => {
      const at = mapToSvg(point.x, point.y);
      // SVG rotate 는 시계방향, ROS yaw 는 반시계방향
      const spin = (-(Number(point.yaw) || 0)).toFixed(1);
      return `<g transform="translate(${at.x.toFixed(1)} ${at.y.toFixed(1)})">`
        + `<g transform="rotate(${spin})">`
        + `<path d="M16 0H30" stroke="#2f6fff" stroke-width="4" stroke-linecap="round"/>`
        + `<path d="M28-7L41 0L28 7Z" fill="#2f6fff"/></g>`
        + `<circle r="17" fill="#2f6fff" stroke="#fff" stroke-width="5"/>`
        + `<text y="4" text-anchor="middle" fill="#fff" font-size="11" font-weight="800">${index + 1}</text>`
        + `</g>`;
    }).join('');
  }

  function toggleWaypointMode() {
    if (!canOperate()) return;
    state.addWaypoint = !state.addWaypoint;
    state.manualPose = false;
    $('#addWaypointMode').classList.toggle('active', state.addWaypoint);
    $('#mapModeHint').textContent = state.addWaypoint
      ? '맵을 클릭해 웨이포인트를 추가하세요. 누른 채 끌면 그 방향이 목표 헤딩이 됩니다.'
      : '맵을 탐색하거나 웨이포인트를 추가하세요.';
  }

  /* ── 목표 헤딩 드래그 ─────────────────────────────────────────────────
   *
   * RViz 의 2D Goal Pose 와 같은 조작이다: 누른 자리가 위치, 끈 방향이 헤딩.
   *
   * 헤딩을 굳이 지정하게 하는 이유는 Nav2 가 그것을 **강제하기 때문**이다.
   * nav2.yaml 의 yaw_goal_tolerance 는 0.20 rad(11.5°)이고, 경로의 마지막
   * pose 가 목표 yaw 를 담은 채 MPPI 의 PathAlignCritic(가중치 최대)로 들어간다.
   * 즉 헤딩을 안 주면 '아무 방향이나'가 아니라 **0° 를 요구한 것**이 된다.
   * 그러니 조작자가 그 값을 볼 수 있어야 하고, 지정할 수 있어야 한다.
   *
   * ⚠ 자세 지정 목표는 이 플랫폼의 알려진 약점이다(docs/control_pipeline.md
   *   §12.5.2). R_min 1.643 m 원호로만 탐색하므로 '가까운 거리에서 큰 자세변화'
   *   에는 해가 없어 계획이 실패한다. 헤딩을 로봇의 접근 방향과 크게 어긋나게
   *   찍으면 그 실패를 만들 수 있다 — 아래 커밋 시점에 각도를 안내한다.
   */
  function goalDragPoint(event) {
    const point = svgPoint(event);
    return { svg: point, map: pixelToMap(point.x, point.y) };
  }

  function beginGoalDrag(event) {
    if (!state.addWaypoint) return;
    if (!canOperate()) return;
    const at = goalDragPoint(event);
    state.goalDrag = { from: at, to: at };
    // 포인터를 캡처해 두면 지도 밖으로 끌고 나가도 이벤트가 계속 온다.
    try { event.currentTarget.setPointerCapture(event.pointerId); } catch (error) { /* 구형 브라우저 */ }
    event.preventDefault();
  }

  function updateGoalDrag(event) {
    if (!state.goalDrag) return;
    state.goalDrag.to = goalDragPoint(event);
    drawGoalPreview(state.goalDrag);
  }

  function drawGoalPreview(drag) {
    const layer = $('#goalPreviewLayer');
    if (!layer) return;
    const from = drag.from.svg;
    const to = drag.to.svg;
    const length = Math.hypot(to.x - from.x, to.y - from.y);
    if (length < GOAL_DRAG_MIN) {
      layer.innerHTML = `<circle cx="${from.x.toFixed(1)}" cy="${from.y.toFixed(1)}" `
        + `r="6" fill="#4ADE9B"/>`;
      return;
    }
    // 미리보기 각도는 SVG 좌표계에서 그대로 잰다(그리기용). 실제로 저장하는
    // yaw 는 아래에서 **지도 좌표로** 다시 계산한다 — y 축 방향이 반대라
    // 화면 각도를 그대로 쓰면 위아래가 뒤집힌다.
    const degrees = Math.atan2(to.y - from.y, to.x - from.x) * 180 / Math.PI;
    const shaft = Math.max(0, length - 13);
    layer.innerHTML = `<g transform="translate(${from.x.toFixed(1)} ${from.y.toFixed(1)}) `
      + `rotate(${degrees.toFixed(1)})">`
      + `<path d="M0 0H${shaft.toFixed(1)}" stroke="#4ADE9B" stroke-width="4" stroke-linecap="round"/>`
      + `<path d="M${(length - 15).toFixed(1)}-8L${length.toFixed(1)} 0L${(length - 15).toFixed(1)} 8Z" fill="#4ADE9B"/>`
      + `</g>`
      + `<circle cx="${from.x.toFixed(1)}" cy="${from.y.toFixed(1)}" r="6" fill="#4ADE9B"/>`;
  }

  function endGoalDrag(event) {
    const drag = state.goalDrag;
    if (!drag) return;
    state.goalDrag = null;
    $('#goalPreviewLayer').innerHTML = '';
    try { event.currentTarget.releasePointerCapture(event.pointerId); } catch (error) { /* 무시 */ }

    const to = goalDragPoint(event);
    const from = drag.from;
    const pulled = Math.hypot(to.svg.x - from.svg.x, to.svg.y - from.svg.y);
    // yaw 는 **지도 좌표로** 잰다. 화면은 y 가 아래로 증가하므로 여기서 재야
    // ROS 규약(반시계 양수)과 맞는다.
    const dragged = pulled >= GOAL_DRAG_MIN;
    const yaw = dragged
      ? Math.atan2(to.map.y - from.map.y, to.map.x - from.map.x) * 180 / Math.PI
      : 0;

    state.waypoints.push({
      id: `${Date.now()}-${Math.random()}`,
      label: `Waypoint ${state.waypoints.length + 1}`,
      x: from.map.x, y: from.map.y, yaw: Number(yaw.toFixed(1)),
      tolerance: Number($('#defaultTolerance').value),
    });
    renderWaypoints();
    renderNavigation();          // 목표가 생기면 '주행 시작' 이 열린다
    toast('웨이포인트를 추가했습니다',
      `x ${from.map.x.toFixed(2)} · y ${from.map.y.toFixed(2)} · yaw ${yaw.toFixed(0)}°`
      + (dragged ? '' : ' (끌면 헤딩을 지정할 수 있습니다)'),
      'success');
  }

  function mapClick(event) {
    // 웨이포인트 추가는 포인터 드래그 흐름이 담당한다 (beginGoalDrag).
    // 여기서 또 처리하면 pointerup 뒤에 오는 click 이 같은 점을 한 번 더 넣는다.
    if (state.addWaypoint) return;
    const point = svgPoint(event);
    if (state.manualPose) {
      // 라이브에서는 여기 올 수 없다 (manualInitialPose 가 막는다). 목업 모드
      // 데모용 경로로만 남긴다 — 라이브 상태를 가짜 pose 로 덮으면 안 된다.
      if (isLive()) { state.manualPose = false; return; }
      const map = pixelToMap(point.x, point.y);
      state.localization.pose = { x: map.x, y: map.y, yaw: 0 };
      state.localization.state = 'WAITING_ICP';
      state.manualPose = false;
      renderLocalization();
      setTimeout(() => completeLocalization(), 1400);
      return;
    }
  }

  /* ── 측위 ────────────────────────────────────────────────────────────
   *
   * 진행 상태는 **로봇이 말해주는 것만** 그린다. 예전에는 이 자리에서
   * setTimeout 으로 프레임 카운터를 돌리고 fitness 0.087 을 지어냈다 —
   * 라이다가 꺼져 있어도 화면은 똑같이 '수렴'까지 갔다.
   *
   * 두 출처가 들어온다.
   *   /rosout 의 [단계] 태그  → 진행 (ingest.addLocalizationLog)
   *   /icp_result 도착        → 수렴 확정 (ingest.onLocalizationConverged)
   * 둘 다 안 오면 화면도 아무 말 하지 않는다. 그게 사실이다.
   */
  const LOC_STAGE_STATE = {
    ACCUM: 'ACCUMULATING', ATTEMPT: 'MATCHING',
    FPFH: 'MATCHING', MATCH: 'MATCHING', TEASER: 'MATCHING',
    GICP: 'WAITING_ICP', CONSISTENCY: 'WAITING_ICP',
  };

  function renderLocalization() {
    const loc = state.localization;
    const chip = $('#localizationChip');
    chip.className = 'state-chip';
    $('#localizationProgress').classList.toggle('hidden',
      !['STARTING', 'ACCUMULATING', 'MATCHING', 'WAITING_ICP'].includes(loc.state));
    const setProgress = (text, percent) => {
      $('#localizationProgressText').textContent = text;
      $('#localizationProgressBar').style.width = `${percent}%`;
    };

    if (loc.state === 'IDLE') {
      chip.classList.add('idle'); chip.textContent = '대기';
      $('#localizationTitle').textContent = '초기위치가 필요합니다';
      $('#localizationDetail').textContent = 'FPFH + TEASER++ 자동 초기화 가능';
    } else if (loc.state === 'STARTING') {
      chip.classList.add('running'); chip.textContent = '기동';
      $('#localizationTitle').textContent = '측위 스택을 띄우는 중';
      $('#localizationDetail').textContent = 'localization.launch.py — 맵과 DB 를 읽습니다.';
      setProgress('waiting for teaser_fpfh_localizer', 12);
    } else if (loc.state === 'ACCUMULATING') {
      chip.classList.add('running'); chip.textContent = '누적 중';
      $('#localizationTitle').textContent = loc.attempts > 1
        ? `${loc.attempts}번째 시도 — 스캔 재누적` : '라이다 프레임 누적 중';
      $('#localizationDetail').textContent = '로봇을 정지 상태로 유지하세요.';
      setProgress(`프레임 ${loc.frame} / ${loc.frameTotal} 누적`,
        (loc.frame / Math.max(1, loc.frameTotal)) * 100);
    } else if (loc.state === 'MATCHING') {
      chip.classList.add('running'); chip.textContent = '매칭';
      $('#localizationTitle').textContent = 'FPFH 대응점 · TEASER++ 전역정합';
      $('#localizationDetail').textContent = '맵 전체에서 현재 스캔의 위치를 찾고 있습니다.';
      // 대응점 수가 문턱을 못 넘는 것이 가장 흔한 실패다. 숫자를 그대로 보인다.
      setProgress(loc.matches == null
        ? 'FPFH correspondence matching'
        : `대응점 ${loc.matches}` + (loc.matchesNeeded ? ` / ${loc.matchesNeeded} 필요` : ''),
        62);
    } else if (loc.state === 'WAITING_ICP') {
      chip.classList.add('warning'); chip.textContent = 'GICP';
      $('#localizationTitle').textContent = loc.needed
        ? `일치 확인 ${loc.accepted} / ${loc.needed}`
        : '지역 GICP 정밀화';
      $('#localizationDetail').textContent =
        '잘못된 단발 결과를 막기 위해 새 스캔에서 같은 답이 나오는지 확인합니다.';
      setProgress(`overlap ${loc.overlap == null ? '—' : `${loc.overlap.toFixed(1)}%`}`
        + ` · rmse ${fmtRmse(loc.rmse)}`, 84);
    } else if (loc.state === 'CONVERGED') {
      chip.classList.add('success'); chip.textContent = '수렴';
      $('#localizationTitle').textContent = '초기위치가 확정되었습니다';
      $('#localizationDetail').textContent = 'map → odom 변환이 연결되었습니다.';
    } else {
      chip.classList.add('warning'); chip.textContent = '중단';
      $('#localizationTitle').textContent = '측위가 종료되었습니다';
      $('#localizationDetail').textContent = '정합 로그에서 거절 사유를 확인하세요.';
    }

    // 버튼은 **서버가 본 슬롯 상태**를 따른다 (loc.running). 화면의 진행 상태로
    // 판단하면 CLI 로 띄운 측위나 혼자 죽은 프로세스를 놓친다.
    // running === null 은 '아직 모른다' 라 목업 모드와 첫 폴링 전을 뜻한다.
    const live = isLive();
    const running = Boolean(loc.running);
    // 웹 밖에서 띄운 것은 내릴 수 없다 — 프로세스 그룹을 우리가 안 갖고 있다.
    // 버튼을 살려두면 눌렀을 때 409 만 받고 왜인지는 모른다.
    const stopButton = $('#stopLocalizationBtn');
    if (stopButton) {
      stopButton.disabled = live && (!running || loc.external);
      stopButton.title = loc.external
        ? '웹 밖에서 기동한 측위입니다 — 띄운 터미널에서 Ctrl-C 하세요'
        : '';
    }
    const startButton = $('#autoLocalization');
    if (startButton) startButton.disabled = live && running;

    $('#localizationMapText').textContent = loc.map || '—';
    $('#fitnessText').textContent = fmtRmse(loc.rmse);
    $('#overlapText').textContent = loc.overlap == null ? '—' : `${loc.overlap.toFixed(1)}%`;
    $('#poseText').textContent = loc.pose
      ? `${loc.pose.x.toFixed(2)}, ${loc.pose.y.toFixed(2)}, ${loc.pose.yaw.toFixed(0)}°`
      : '미확정';
    $('#confidenceText').textContent = loc.state === 'CONVERGED' ? '높음' : '—';
    $('#localizationModeText').textContent = 'FPFH + TEASER++';
  }

  const fmtRmse = (value) => (value == null ? '—' : value.toFixed(3));

  async function autoLocalization() {
    const api = requireCmd();
    if (!api) return;
    if (!canOperate()) return;
    if (state.systemState === 'MAPPING') {
      toast('매핑 중에는 측위할 수 없습니다',
        'FAST-LIO 가 두 개 뜨면 오도메트리와 TF 가 겹칩니다. 매핑을 먼저 끝내세요.',
        'warning');
      return;
    }
    if (state.localization.running) {
      toast('측위가 이미 실행 중입니다', '다시 시도하려면 재측위를 쓰세요.', 'warning');
      return;
    }

    const button = $('#autoLocalization');
    setButtonBusy(button, true, '기동 중…');
    state.systemState = 'LOCALIZING';
    state.localization = {
      ...state.localization,
      state: 'STARTING', frame: 0, attempts: 0, accepted: 0, needed: 0,
      matches: null, matchesNeeded: null, rmse: null, overlap: null, pose: null,
    };
    renderLocalization(); renderGlobal();

    try {
      // 맵 이름을 보내지 않는다 — 활성 맵의 진실은 서버에 있다.
      const result = await api.startLocalization({});
      state.localization.map = result.map;
      state.localization.running = true;
      state.locLogOffset = 0;
      state.localizationLogs = [];
      renderLocalization();          // 버튼 상태를 즉시 뒤집는다 (폴링 3초를 기다리지 않는다)
      renderLocalizationLogs();
      startLocalizationLogPoll();
      addLog('INFO', 'alm_web_backend',
        `측위 기동: 맵 ${result.map} (feature ${result.summary?.db_features ?? '?'}개, `
        + `${result.accum_frames}프레임 누적)`);
      for (const note of result.notes || []) addAlarm('warning', '측위 주의', note);
      toast('측위를 시작했습니다', result.message, 'success');
    } catch (error) {
      // 409 는 오류가 아니라 '지금은 못 한다'는 안내다. 이유가 본문에 있다.
      state.systemState = 'IDLE';
      state.localization.state = 'IDLE';
      state.localization.running = false;
      renderLocalization(); renderGlobal();
      addLog('WARN', 'alm_web_backend', `측위 기동 거부: ${error.message}`);
      toast('측위를 시작할 수 없습니다', error.message,
        error.status === 409 ? 'warning' : 'error');
    } finally {
      setButtonBusy(button, false);
    }
  }

  /** 슬롯을 실제로 내린다. 확인창 없음 — 부르는 쪽이 책임진다. */
  async function stopLocalizationProcess(button) {
    const api = requireCmd();
    if (!api) return false;
    setButtonBusy(button, true, '종료 중…');
    try {
      // 서버가 프로세스 그룹에 SIGINT → SIGTERM → SIGKILL 을 보낸다. launch 가
      // 자식 노드를 역순으로 정리할 시간을 주므로 최대 15초 걸린다.
      await api.stopLocalization();
      // 서버가 이미 내린 것이 확실하다. health 폴링(3초)을 기다리면 그동안
      // '측위 중단' 버튼이 계속 눌리는 상태로 남는다.
      state.localization.running = false;
      stopLocalizationLogPoll();
      renderLocalization();
      addLog('INFO', 'alm_web_backend', 'localization.launch.py 를 종료했습니다.');
      return true;
    } catch (error) {
      toast('측위 종료 실패', error.message, 'error');
      return false;
    } finally {
      setButtonBusy(button, false);
    }
  }

  /**
   * '측위 중단' 버튼.
   *
   * 이미 수렴한 뒤에 내리는 것은 단순한 취소가 아니다. 이 launch 에는
   * transform_publisher(map→odom)와 fast_lio(odom→base_link)가 같이 들어 있어서,
   * 내리면 **로봇이 자기 위치를 통째로 잃는다.** 주행 중이면 Nav2 도 같이 멈춘다.
   * 정합을 기다리다 취소하는 것과는 결과가 다르므로 그때만 확인을 받는다.
   */
  async function stopLocalizationClicked() {
    if (!canOperate()) return;
    const button = $('#stopLocalizationBtn');
    if (state.localization.state === 'CONVERGED') {
      const ok = await confirmModal({
        title: '측위를 중단할까요?',
        body: '이미 초기위치가 잡혀 있습니다. 중단하면 <b>map → odom 과 '
          + 'odom → base_link 가 모두 사라져 로봇이 자기 위치를 잃습니다.</b> '
          + '다시 쓰려면 처음부터 정합해야 합니다.',
        confirm: '중단',
        kicker: 'STOP',
      });
      if (!ok) return;
    }
    if (!await stopLocalizationProcess(button)) return;
    toast('측위를 중단했습니다', 'localization.launch.py 를 종료했습니다.');
  }

  /**
   * /icp_result 가 도착했다 (또는 목업 모드의 수동 지정).
   *
   * pose 인자가 있으면 실측이다. 없으면 목업 경로이므로 아무것도 지어내지 않고
   * 상태만 바꾼다.
   */
  function completeLocalization(pose) {
    const loc = state.localization;
    loc.state = 'CONVERGED';
    if (pose) loc.pose = pose;
    if (state.systemState === 'LOCALIZING') state.systemState = 'IDLE';
    $('#candidateLayer')?.classList.add('hidden-layer');
    renderLocalization(); renderGlobal();
    const detail = loc.pose
      ? `x ${loc.pose.x.toFixed(2)} · y ${loc.pose.y.toFixed(2)} · yaw ${loc.pose.yaw.toFixed(0)}°`
      : '';
    addAlarm('info', '초기위치 확정', detail);
    toast('초기위치가 확정되었습니다', detail, 'success');
  }

  /** ingest 가 /icp_result 로 부른다. */
  function onLocalizationConverged(pose) {
    completeLocalization(pose);
  }

  function manualInitialPose() {
    if (!canOperate()) return;
    if (isLive()) {
      // 붙일 대상이 없다. 이 브랜치의 teaser_fpfh_localizer 는 /initialpose 를
      // 구독하지 않는다 (구독하는 것은 안 쓰는 icp_node 다). 눌러도 아무 일이
      // 없는 버튼을 '되는 척' 두지 않는다.
      toast('수동 초기위치는 아직 연결되지 않았습니다',
        'FPFH + TEASER++ 는 초기 추정 없이 전역에서 찾습니다 — 자동 탐색을 쓰세요.',
        'warning');
      return;
    }
    state.manualPose = true; state.addWaypoint = false;
    $('#addWaypointMode').classList.remove('active');
    $('#mapModeHint').textContent = '맵을 클릭해 초기 위치를 지정하세요. 방향은 0°로 모의 적용됩니다.';
    toast('수동 초기위치 지정 모드', '맵에서 로봇 위치를 클릭하세요.');
  }

  /* ── 정합 로그 ───────────────────────────────────────────────────────
   * 출처가 둘이고 잡는 실패가 다르다.
   *   ros     : /rosout — 정상 동작 중의 진행. 노드가 떠야만 나온다.
   *   process : launch 의 stdout/stderr — 기동 실패, 맵 로딩 중 크래시, PCL 예외.
   *             ROS 로그가 **비어 있는 것 자체가 증상**일 때 볼 곳이다.
   */
  const LOC_LOG_MAX = 200;

  function addLocalizationLog(entry) {
    const line = { time: nowTime(), source: 'ros', ...entry };
    applyLocalizationProgress(line);

    // [ACCUM] 은 목록에 넣지 않는다. 시도마다 열 줄이 똑같이 찍혀서, 200줄
    // 버퍼가 20초 만에 그것들로만 채워진다 — 정작 읽어야 할 거절 사유
    // ([MATCH]/[TEASER]/[GICP]) 가 밀려나간다. 프레임 진행은 진행바가 보여준다.
    if (line.stage === 'ACCUM') return;

    state.localizationLogs.push(line);
    if (state.localizationLogs.length > LOC_LOG_MAX) state.localizationLogs.shift();
    if (state.locLogSource === 'ros') renderLocalizationLogs();
  }

  /** 로그 한 줄에서 진행 상태와 수치를 뽑는다. 화면이 지어내지 않는 유일한 근거. */
  function applyLocalizationProgress(line) {
    const loc = state.localization;
    const text = line.text || '';
    const next = LOC_STAGE_STATE[line.stage];
    if (next && !['CONVERGED'].includes(loc.state)) loc.state = next;

    // "[ACCUM] frame 6/10, points=121083 (robot must remain stationary)"
    const frame = /frame (\d+)\/(\d+)/.exec(text);
    if (frame) {
      loc.frame = Number(frame[1]);
      loc.frameTotal = Number(frame[2]);
    }
    // "[MATCH] mutual+ratio matches=9 (required >= 20), time=.."
    const matches = /matches=(\d+)(?:\s*\(required >= (\d+)\))?/.exec(text);
    if (matches) {
      loc.matches = Number(matches[1]);
      if (matches[2]) loc.matchesNeeded = Number(matches[2]);
    }
    // "[TEASER] matches=.. clique=.. overlap=62.4% rmse=0.183 time=.."
    const overlap = /overlap=([\d.]+)%/.exec(text);
    if (overlap) loc.overlap = Number(overlap[1]);
    const rmse = /rmse=([\d.]+)/.exec(text);
    if (rmse) loc.rmse = Number(rmse[1]);
    // "[CONSISTENCY] accepted 1/2"
    const consistency = /accepted (\d+)\/(\d+)/.exec(text);
    if (consistency) {
      loc.accepted = Number(consistency[1]);
      loc.needed = Number(consistency[2]);
    }
    // "[ATTEMPT 3] rejected; accumulating a fresh scan"
    const attempt = /^\[ATTEMPT (\d+)\]/.exec(text);
    if (attempt) {
      loc.attempts = Number(attempt[1]);
      loc.accepted = 0;
    }
    if (/localization succeeded/i.test(text)) loc.state = 'CONVERGED';
    renderLocalization();
  }

  function renderLocalizationLogs() {
    const box = $('#locLogWindow');
    if (!box) return;
    const rows = state.localizationLogs.filter((line) =>
      (line.source || 'ros') === state.locLogSource);
    $('#locLogCount').textContent = `${rows.length} 줄`;

    if (!rows.length) {
      box.innerHTML = `<div class="loc-log-empty">${state.locLogSource === 'ros'
        ? '아직 측위 노드 로그가 없습니다. 측위를 시작하면 여기에 단계별로 흐릅니다.<br>'
          + '기동했는데도 계속 비어 있다면 <b>프로세스 출력</b> 쪽을 보세요 — 노드가 뜨기 전에 죽으면 ROS 로그는 한 줄도 안 남습니다.'
        : '프로세스 출력이 없습니다. 측위를 한 번도 기동하지 않았거나 로그 파일이 아직 비어 있습니다.'}</div>`;
      return;
    }

    box.innerHTML = rows.map((line) => {
      const level = (line.level || 'INFO').toLowerCase();
      const ok = /succeeded|converged/i.test(line.text || '');
      const stageClass = line.stage ? ` t-${line.stage.toLowerCase()}` : '';
      const tag = line.stage || (line.source === 'process' ? 'OUT' : level.toUpperCase());
      return `<div class="loc-line ${ok ? 'ok' : esc(level)}${stageClass}">
        <div class="loc-head"><time>${esc(line.time)}</time>
        <span class="loc-tag">${esc(tag)}</span>
        <span class="loc-node">${esc(line.node || '')}</span></div>
        <p>${esc(line.text || '')}</p></div>`;
    }).join('');

    // 사용자가 위로 스크롤해 읽는 중이면 밀지 않는다. 매핑 탭 로그창은 무조건
    // 맨 아래로 밀어서, 읽으려고 올리면 새 줄이 올 때마다 튕겨 내려간다.
    if (state.locLogFollow) box.scrollTop = box.scrollHeight;
  }

  function startLocalizationLogPoll() {
    stopLocalizationLogPoll();
    state.locLogTimer = setInterval(pollLocalizationLog, 1000);
    pollLocalizationLog();
  }

  function stopLocalizationLogPoll() {
    clearInterval(state.locLogTimer);
    state.locLogTimer = null;
  }

  async function pollLocalizationLog() {
    const api = cmd();
    if (!api || !api.hasToken()) return;
    try {
      const result = await api.localizationLog(state.locLogOffset);
      state.locLogOffset = result.offset ?? state.locLogOffset;
      // [ACCUM] 은 초당 열 줄씩 찍힌다 (실측 18줄/s). 걸러내지 않으면 200줄
      // 버퍼가 11초 만에 그것들로 채워져, 이 뷰의 존재 이유인 **기동 실패
      // 출력**이 맨 위로 밀려나간다. 프레임 진행은 진행바가 이미 보여준다.
      for (const text of (result.lines || []).filter((line) => !line.includes('[ACCUM]'))) {
        state.localizationLogs.push({
          time: nowTime(), source: 'process', node: 'launch',
          level: /error|Traceback|what\(\):|terminate called/i.test(text) ? 'ERROR' : 'INFO',
          stage: '', text,
        });
      }
      if (state.localizationLogs.length > LOC_LOG_MAX) {
        state.localizationLogs.splice(0, state.localizationLogs.length - LOC_LOG_MAX);
      }
      if ((result.lines || []).length && state.locLogSource === 'process') {
        renderLocalizationLogs();
      }
      // 프로세스가 내려갔으면 더 볼 것이 없다. 마지막 한 번을 읽고 멈춘다.
      if (!result.running && !(result.lines || []).length) stopLocalizationLogPoll();
    } catch {
      // 백엔드가 잠깐 끊긴 것일 수 있다. 다음 주기에 다시 시도한다.
    }
  }

  /**
   * 서버가 본 localization 슬롯 상태가 바뀌었다 (main.js 의 health 폴링).
   * onSlamRunningChange 와 같은 이유다 — CLI 나 다른 탭에서 시작한 측위도
   * 화면이 알아야 한다.
   */
  function onLocalizationRunningChange(running, slot, { external = false, nodes = [] } = {}) {
    // 첫 폴링에서 null → false 로 확정되는 것은 '종료'가 아니라 '원래 안 돌고
    // 있었다'다. 여기서 종료 처리를 하면 페이지를 열 때마다 로그에 거짓
    // 종료가 한 줄씩 남는다.
    const first = state.localization.running === null;
    const was = state.localization.running;
    state.localization.running = running;
    state.localization.external = external;
    if (first && !running) return;
    if (running === was) { renderLocalization(); return; }   // external 만 바뀐 경우
    if (running) {
      if (state.localization.state === 'IDLE') {
        state.localization.state = 'ACCUMULATING';
        addLog('WARN', 'alm_web_backend', external
          ? `웹 밖에서 시작된 측위를 감지했습니다 (${nodes.join(', ')}). `
            + '웹에서는 내릴 수 없습니다 — 띄운 터미널에서 Ctrl-C 하세요.'
          : `이 탭 밖에서 시작된 측위를 감지했습니다 (pid=${slot?.pid}).`);
      }
      // 기동 인자에 실제로 쓰인 맵이 진실이다. active.yaml 은 그 사이 바뀌었을 수 있다.
      const arg = (slot?.args || []).find((item) => item.startsWith('map_pcd:='));
      if (arg) {
        const parts = arg.split('/');
        state.localization.map = parts[parts.length - 2] || state.localization.map;
      }
      startLocalizationLogPoll();
    } else {
      stopLocalizationLogPoll();
      if (state.localization.state !== 'CONVERGED') state.localization.state = 'STOPPED';
      if (state.systemState === 'LOCALIZING') state.systemState = 'IDLE';
      addLog('INFO', 'alm_web_backend', '측위 프로세스가 종료되었습니다.');
    }
    renderLocalization(); renderGlobal();
  }

  /**
   * 재측위 = 측위 슬롯 재기동.
   *
   * teaser_fpfh_localizer 는 성공하면 finished_ 플래그로 영구히 유휴 상태가
   * 되고, 리셋 서비스가 없다. 그래서 다시 찾게 하려면 프로세스를 새로 띄우는
   * 수밖에 없는데, 같은 launch 에 fast_lio 가 들어 있어서 **추적 오도메트리도
   * 함께 리셋된다.** 조작자가 그걸 모르고 누르면 안 된다.
   */
  async function relocalize() {
    if (!canOperate()) return;
    if (state.nav.state === 'RUNNING') {
      toast('주행 중 재측위할 수 없습니다', '주행을 중단하거나 일시정지한 뒤 시도하세요.', 'warning');
      return;
    }
    if (isLive() && state.localization.running) {
      const ok = await confirmModal({
        title: '측위를 다시 시작할까요?',
        body: '측위 스택을 통째로 재기동합니다. teaser_fpfh_localizer 에는 리셋 '
          + '기능이 없어서 프로세스를 새로 띄우는 방법뿐이고, 같은 launch 에 있는 '
          + 'FAST-LIO 도 함께 재시작되어 <b>지금까지의 추적 오도메트리가 초기화</b>됩니다.',
        confirm: '재측위',
      });
      if (!ok) return;
      if (!await stopLocalizationProcess($('#relocalize'))) return;
      // 슬롯이 완전히 내려간 뒤에 다시 띄운다. stop() 은 SIGINT→SIGTERM 까지
      // 기다렸다가 돌아오므로 여기서 추가로 잴 것은 없다.
      state.localization.running = false;
    }
    state.localization = {
      ...state.localization,
      state: 'IDLE', frame: 0, attempts: 0, accepted: 0, needed: 0,
      matches: null, matchesNeeded: null, rmse: null, overlap: null, pose: null,
    };
    renderLocalization();
    autoLocalization();
  }

  // ── 자율주행 ────────────────────────────────────────────────────────
  // 화면 상태의 진실은 **서버에 있다.** 아래 state.nav 는 /api/navigation 의
  // 캐시일 뿐이고, 버튼을 누른 직후를 빼면 값을 여기서 만들지 않는다.
  //
  // 목업은 setInterval 로 progress 를 0.8%씩 올렸다. 로봇이 서 있어도 화면은
  // 100% 를 향해 꾸준히 올라갔고, 도착하지 않았는데 '미션 완료' 토스트가 떴다.
  // 그 종류의 거짓말을 없애는 것이 이 절의 요지다 — 진척은 Nav2 피드백에서만
  // 온다. 피드백이 없는 값은 지어내지 않고 '—' 로 둔다.
  //
  // 두 층을 구분해야 한다:
  //   스택(navigation.launch.py)  기동에 수십 초. map_server + 측위 + Nav2
  //   목표(NavigateToPose 등)      전송에 수십 ms. 이미 뜬 Nav2 에 보낸다
  // '주행 시작' 버튼 하나가 상태에 따라 둘 중 하나를 한다 (navStartPlan 참조).

  const NAV_STATE_LABEL = {
    idle: 'IDLE', pending: 'RUNNING', active: 'RUNNING', paused: 'PAUSED',
    succeeded: 'COMPLETED', failed: 'FAILED', canceled: 'IDLE',
  };

  /** /api/navigation 응답을 화면 상태로 옮긴다. 값을 만들어내지 않는다. */
  function applyNavStatus(payload) {
    const mission = payload?.mission || {};
    const nav = state.nav;
    const previous = nav.serverState;

    // 명령 응답({mission} 만 들어 있음)으로 부를 때는 스택 판정을 건드리지
    // 않는다. 목표를 보냈다는 사실이 '누가 스택을 띄웠는가'를 바꾸지는 않으므로,
    // 여기서 지어내면 다음 폴링(1.5초)까지 external 표시가 틀린 채로 남는다.
    if (payload && ('process' in payload || 'nodes' in payload)) {
      const nodes = payload.nodes || [];
      nav.stackRunning = Boolean(payload.process?.running) || nodes.length > 0;
      nav.stackExternal = !payload.process?.running && nodes.length > 0;
    }
    nav.ready = Boolean(mission.action_ready && mission.tf_ready);
    nav.actionReady = Boolean(mission.action_ready);
    nav.tfReady = Boolean(mission.tf_ready);
    nav.serverState = mission.state || 'idle';
    nav.state = NAV_STATE_LABEL[nav.serverState] || 'IDLE';
    nav.kind = mission.kind || '';
    nav.current = Number(mission.index) || 0;
    nav.total = Number(mission.total) || 0;
    nav.message = mission.message || '';
    nav.distance = mission.distance_remaining_m;
    nav.eta = mission.eta_sec;
    nav.recoveries = Number(mission.recoveries) || 0;
    nav.estimate = mission.distance_estimate_m;

    // 진척률. **관측된 것에서만** 만든다.
    //   단일 목표  : 첫 피드백의 남은거리를 분모로 삼는다 (d0 는 서버가 모른다)
    //   웨이포인트 : 도달한 목표 수 / 전체
    if (nav.serverState === 'idle') {
      nav.progress = 0; nav.distance0 = null;
    } else if (nav.kind === 'pose') {
      if (typeof nav.distance === 'number') {
        if (nav.distance0 == null || nav.distance > nav.distance0) nav.distance0 = nav.distance;
        nav.progress = nav.distance0 > 0
          ? clamp((1 - nav.distance / nav.distance0) * 100, 0, 100) : 0;
      }
    } else if (nav.total > 0) {
      nav.progress = clamp((nav.current / nav.total) * 100, 0, 100);
    }
    if (nav.serverState === 'succeeded') nav.progress = 100;

    // 시스템 상태는 주행 중일 때만 NAVIGATING 이다. 매핑/측위가 쓰는 것과
    // 같은 칸이므로 여기서 함부로 IDLE 로 되돌리지 않는다.
    if (nav.state === 'RUNNING') state.systemState = 'NAVIGATING';
    else if (state.systemState === 'NAVIGATING') state.systemState = 'IDLE';

    // 미션이 끝난 순간에만 알린다. 폴링마다 토스트를 띄우면 안 된다.
    if (previous && previous !== nav.serverState) {
      if (nav.serverState === 'succeeded') {
        addAlarm('info', '미션 완료', nav.message || '모든 목표에 도달했습니다.');
        toast('주행 미션이 완료되었습니다', nav.message, 'success');
      } else if (nav.serverState === 'failed') {
        addAlarm('warning', '자율주행 실패', nav.message);
        toast('자율주행이 실패했습니다', nav.message, 'error');
      }
    }
    renderNavigation(); renderGlobal();
  }

  /** 폴링. 화면이 열려 있는 동안만 돈다 — 서버 부하보다 '화면이 늦는' 쪽이 나쁘다. */
  function startNavPoll() {
    if (state.nav.pollTimer) return;
    const tick = async () => {
      const api = cmd();
      if (!api || !api.hasToken()) return;
      try {
        applyNavStatus(await api.navigationStatus());
      } catch (error) {
        // 폴링 실패는 조용히 넘긴다. 백엔드 생존 표시는 HUD 가 따로 한다.
      }
    };
    tick();
    state.nav.pollTimer = setInterval(tick, 1500);
  }

  function renderNavigation() {
    const nav = state.nav;
    const chip = $('#navStateChip');
    chip.className = 'state-chip';
    chip.textContent = nav.state;
    chip.classList.add(nav.state === 'RUNNING' ? 'running'
      : nav.state === 'PAUSED' || nav.state === 'FAILED' ? 'warning'
        : nav.state === 'COMPLETED' ? 'success' : 'idle');
    chip.title = nav.message || '';

    $('#missionPercent').textContent = `${Math.round(nav.progress)}%`;
    $('#missionProgressRing').style.setProperty('--progress', `${nav.progress * 3.6}deg`);
    $('#currentGoalText').textContent = nav.total && nav.state !== 'IDLE'
      ? `${Math.min(nav.current + 1, nav.total)} / ${nav.total}` : '—';

    // 남은 거리와 ETA 는 NavigateToPose 피드백에만 있다. FollowWaypoints 는
    // current_waypoint 하나만 주므로, 웨이포인트 미션에서는 알 수 없다.
    // 직선 합(estimate)을 여기 쓰면 안 된다 — Hybrid-A* 실경로는 항상 그보다
    // 길어서 '남은 거리'로 읽히는 순간 거짓이 된다. 참고값으로 title 에만 둔다.
    const distanceNode = $('#remainingDistance');
    distanceNode.textContent = typeof nav.distance === 'number'
      ? `${nav.distance.toFixed(1)} m` : '—';
    distanceNode.title = typeof nav.distance === 'number' ? ''
      : (nav.estimate != null
        ? `웨이포인트 미션은 Nav2 가 남은 거리를 주지 않습니다 (직선 합 ${nav.estimate} m)`
        : '');
    $('#etaText').textContent = typeof nav.eta === 'number' ? `${Math.round(nav.eta)} s` : '—';

    const actual = state.driveMode?.effective || '—';
    $('#navDriveMode').textContent = `${nav.mode} / ${actual}`;

    const plan = navStartPlan();
    const startButton = $('#startNavigation');
    startButton.textContent = plan.label;
    startButton.disabled = !plan.enabled;
    startButton.title = plan.reason;

    const busy = ['RUNNING', 'PAUSED'].includes(nav.state);
    $('#pauseNavigation').disabled = !busy;
    $('#pauseNavigation').textContent = nav.state === 'PAUSED' ? '재개' : '일시정지';
    $('#cancelNavigation').disabled = !busy && !nav.stackRunning;
    $('#cancelNavigation').title = busy
      ? '진행 중인 목표를 취소합니다'
      : nav.stackRunning ? '자율주행 스택(Nav2 + 측위)을 내립니다' : '';
  }

  /**
   * '주행 시작' 이 지금 무엇을 해야 하는지. 라벨·활성화·사유를 함께 돌려준다.
   *
   * 한 버튼이 두 가지 일을 하는 것은 조작 순서가 **하나뿐이기 때문**이다:
   * 스택을 띄우고 → 정합을 기다리고 → 목표를 보낸다. 버튼을 둘로 나누면
   * 조작자가 순서를 고를 수 있는 것처럼 보이는데, 실제로는 못 고른다.
   */
  function navStartPlan() {
    const nav = state.nav;
    if (!state.hasControl) return { action: null, label: '주행 시작', enabled: false, reason: '제어권이 없습니다' };
    if (['RUNNING', 'PAUSED'].includes(nav.state)) {
      return { action: null, label: '주행 중', enabled: false, reason: '이미 주행 중입니다' };
    }
    if (!nav.stackRunning) {
      if (state.systemState === 'MAPPING') {
        return { action: null, label: '자율주행 기동', enabled: false,
          reason: '매핑 중에는 기동할 수 없습니다' };
      }
      if (state.localization.running) {
        return { action: null, label: '자율주행 기동', enabled: false,
          reason: '측위가 따로 떠 있습니다 — 자율주행 스택이 측위를 포함하므로 먼저 측위를 중단하세요' };
      }
      return { action: 'stack', label: '자율주행 기동', enabled: true,
        reason: 'Nav2 + 측위 스택을 띄웁니다' };
    }
    if (nav.stackExternal && !nav.actionReady) {
      return { action: null, label: '정합 대기 중…', enabled: false,
        reason: '웹 밖에서 기동한 스택입니다' };
    }
    if (!nav.actionReady) {
      return { action: null, label: '기동 중…', enabled: false,
        reason: 'Nav2 액션 서버가 아직 광고되지 않았습니다' };
    }
    if (!nav.tfReady) {
      return { action: null, label: '정합 대기 중…', enabled: false,
        reason: '초기위치가 아직 안 잡혔습니다 (map→odom TF 없음). 로봇을 정지시켜 두세요' };
    }
    if (!state.waypoints.length) {
      return { action: null, label: '주행 시작', enabled: false,
        reason: '맵을 클릭해 목표를 하나 이상 추가하세요' };
    }
    return { action: 'goal', label: '주행 시작', enabled: true, reason: '' };
  }

  /**
   * 화면의 웨이포인트를 서버로 보낼 목록으로 편다.
   *
   * 반복/순환은 Nav2 기능이 아니다. FollowWaypoints 는 목록을 한 번 훑고 끝나므로,
   * 반복은 목록을 그만큼 늘려서 보내는 것으로 구현한다. 서버가 대신 반복 호출을
   * 하게 만들지 않은 이유는, 그러면 '지금 몇 바퀴째인가' 라는 상태가 서버와
   * 화면 양쪽에 생기기 때문이다 — 목록 하나면 index 하나로 전부 표현된다.
   */
  function navMissionPoints() {
    const laps = clamp(Number($('#repeatCount').value) || 1, 1, 99);
    const loop = $('#loopMission').checked;
    const base = state.waypoints.map((point) => ({
      x: point.x, y: point.y, yaw_deg: Number(point.yaw) || 0, label: point.label,
    }));
    const points = [];
    for (let lap = 0; lap < laps; lap += 1) {
      points.push(...base);
      // 순환은 '마지막 목표 뒤에 첫 목표를 다시' 다. 마지막 바퀴에도 붙여야
      // 출발점으로 돌아와서 끝난다 (그게 순환 주행의 뜻이다).
      if (loop) points.push({ ...base[0] });
    }
    return points;
  }

  async function startNavigation() {
    const api = requireCmd();
    if (!api) return;
    const plan = navStartPlan();
    if (!plan.action) {
      if (plan.reason) toast('자율주행을 시작할 수 없습니다', plan.reason, 'warning');
      return;
    }
    const button = $('#startNavigation');

    if (plan.action === 'stack') {
      setButtonBusy(button, true, '기동 중…');
      try {
        const result = await api.startNavigationStack({});
        state.nav.stackRunning = true;
        addLog('INFO', 'alm_web_backend',
          `자율주행 기동: 맵 ${result.map} (feature ${result.summary?.db_features ?? '?'}개, `
          + `${result.accum_frames}프레임 누적)`);
        for (const note of result.notes || []) addAlarm('warning', '자율주행 주의', note);
        toast('자율주행 스택을 기동했습니다', result.message, 'success');
        startNavLogPoll();
      } catch (error) {
        addLog('WARN', 'alm_web_backend', `자율주행 기동 거부: ${error.message}`);
        toast('자율주행을 기동할 수 없습니다', error.message,
          error.status === 409 ? 'warning' : 'error');
      } finally {
        setButtonBusy(button, false);
        renderNavigation();
      }
      return;
    }

    const points = navMissionPoints();
    setButtonBusy(button, true, '전송 중…');
    try {
      const result = await api.sendGoal(points);
      state.nav.distance0 = null;
      applyNavStatus({ mission: result.mission });
      addAlarm('info', '자율주행 시작', `${points.length}개 목표`);
      addLog('INFO', 'alm_web_backend', `목표 전송: ${points.length}개 `
        + `(직선 ${result.mission?.distance_estimate_m ?? '?'} m)`);
      toast('자율주행을 시작했습니다', result.message, 'success');
    } catch (error) {
      addLog('WARN', 'alm_web_backend', `목표 거부: ${error.message}`);
      toast('목표를 보낼 수 없습니다', error.message,
        error.status === 409 ? 'warning' : 'error');
    } finally {
      setButtonBusy(button, false);
      renderNavigation();
    }
  }

  async function pauseNavigation() {
    const api = requireCmd();
    if (!api) return;
    if (!canOperate()) return;
    const button = $('#pauseNavigation');
    const resuming = state.nav.state === 'PAUSED';
    setButtonBusy(button, true, resuming ? '재개 중…' : '정지 중…');
    try {
      const result = resuming ? await api.resumeNavigation() : await api.pauseNavigation();
      applyNavStatus({ mission: result.mission });
      addLog('INFO', 'alm_web_backend', result.message);
      toast(resuming ? '주행을 재개했습니다' : '주행을 일시정지했습니다',
        result.message, resuming ? 'success' : 'warning');
    } catch (error) {
      toast(resuming ? '재개 실패' : '일시정지 실패', error.message,
        error.status === 409 ? 'warning' : 'error');
    } finally {
      setButtonBusy(button, false);
      renderNavigation();
    }
  }

  /**
   * 미션이 돌고 있으면 목표를 취소하고, 아니면 스택을 내린다.
   *
   * ⚠ 이 버튼은 **비상정지가 아니다.** 취소 요청이 Nav2 를 거쳐 컨트롤러까지
   *   가는 데 시간이 걸리고, 그 사이 로봇은 마지막 명령으로 움직인다.
   *   지금 당장 세워야 하면 E-STOP 이다.
   */
  async function cancelNavigation() {
    const api = requireCmd();
    if (!api) return;
    if (!canOperate()) return;
    const busy = ['RUNNING', 'PAUSED'].includes(state.nav.state);
    const button = $('#cancelNavigation');

    if (!busy) {
      if (!state.nav.stackRunning) return;
      if (state.nav.stackExternal) {
        toast('웹 밖에서 기동한 스택입니다', '띄운 터미널에서 Ctrl-C 하세요.', 'warning');
        return;
      }
      if (!confirm('자율주행 스택(Nav2 + 측위)을 내립니다. 계속할까요?')) return;
      setButtonBusy(button, true, '종료 중…');
      try {
        await api.stopNavigationStack();
        state.nav.stackRunning = false;
        stopNavLogPoll();
        addLog('INFO', 'alm_web_backend', 'navigation.launch.py 를 종료했습니다.');
        toast('자율주행 스택을 내렸습니다', '', 'warning');
      } catch (error) {
        toast('종료 실패', error.message, 'error');
      } finally {
        setButtonBusy(button, false);
        renderNavigation();
      }
      return;
    }

    setButtonBusy(button, true, '중단 중…');
    try {
      const result = await api.cancelNavigation();
      applyNavStatus({ mission: result.mission });
      addLog('WARN', 'alm_web_backend', '자율주행 미션을 중단했습니다.');
      toast('자율주행을 중단했습니다',
        '목표를 취소했습니다. 즉시 정지가 필요하면 E-STOP 을 쓰세요.', 'warning');
    } catch (error) {
      toast('중단 실패', error.message, 'error');
    } finally {
      setButtonBusy(button, false);
      renderNavigation();
    }
  }

  /** 자율주행 슬롯 로그 tail. 측위와 같은 이유다 — /rosout 에 안 나오는 실패가 있다. */
  function startNavLogPoll() {
    if (state.navLogTimer) return;
    state.navLogOffset = 0;
    state.navLogTimer = setInterval(async () => {
      const api = cmd();
      if (!api || !api.hasToken()) return;
      try {
        const snapshot = await api.navigationLog(state.navLogOffset || 0);
        state.navLogOffset = snapshot.offset ?? state.navLogOffset;
        for (const line of snapshot.lines || []) addLog('INFO', 'navigation', line);
        if (!snapshot.running) stopNavLogPoll();
      } catch (error) {
        stopNavLogPoll();
      }
    }, 1500);
  }

  function stopNavLogPoll() {
    clearInterval(state.navLogTimer);
    state.navLogTimer = null;
  }

  function addAlarm(type, title, detail) {
    state.alarms.unshift({ type, title, detail, time: nowTime() });
    state.alarms = state.alarms.slice(0, 8);
    renderAlarms();
  }

  function renderAlarms() {
    $('#alarmList').innerHTML = state.alarms.length ? state.alarms.map((alarm) => `
      <div class="alarm-item ${esc(alarm.type)}"><i>${alarm.type === 'critical' ? '!' : alarm.type === 'warning' ? '△' : 'i'}</i><div><strong>${esc(alarm.title)}</strong><small>${esc(alarm.detail)}</small></div><time>${esc(alarm.time)}</time></div>`).join('') : '<div class="alarm-empty">기록된 이벤트가 없습니다.</div>';
  }

  function renderManual() {
    const manual = state.manual;
    const available = manual.enabled && state.hasControl && !state.estop && state.systemState === 'MANUAL_CONTROL';
    $('#manualLockOverlay').classList.toggle('hidden', available);
    $('#enterManual').classList.toggle('hidden', manual.enabled);
    $('#exitManual').classList.toggle('hidden', !manual.enabled);
    $('#manualStatusText').textContent = manual.enabled ? '모터 활성 · 데드맨 입력 대기' : '수동주행 비활성';
    $('#motorDot').className = `status-dot ${manual.enabled ? 'ok' : ''}`;
    $('#motorState').textContent = manual.enabled ? '활성' : '비활성';
    $('#crabNotice').classList.toggle('hidden', manual.mode !== 'crab');
    $$('#manualModeSelector button').forEach((button) => button.classList.toggle('active', button.dataset.mode === manual.mode));
    $$('#speedMultiplier button').forEach((button) => button.classList.toggle('active', Number(button.dataset.value) / 100 === manual.multiplier));

    const mode = manual.mode;
    const allowed = {
      normal: ['forward', 'left', 'stop', 'right', 'reverse'],
      spin: ['left', 'stop', 'right'],
      crab: [],
      auto: ['forward', 'left', 'stop', 'right', 'reverse'],
    }[mode];
    $$('.drive-button').forEach((button) => {
      button.classList.toggle('hidden', !allowed.includes(button.dataset.command));
      button.disabled = !available;
    });
  }

  function enterManual() {
    if (!canOperate()) return;
    if (state.systemState !== 'IDLE') {
      toast('수동주행에 진입할 수 없습니다', `현재 상태: ${state.systemState}`, 'warning'); return;
    }
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">MOTOR ENABLE</p><h2>웹 수동주행을 시작할까요?</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">차량 주변을 확인하고 외부 조이스틱이 중립인지 확인하세요. 버튼을 누르고 있는 동안에만 속도 명령이 전송됩니다.</p>
      <div class="safety-check-list"><div><i class="status-dot ok"></i><span>제어권</span><strong>보유</strong></div><div><i class="status-dot ok"></i><span>E-STOP</span><strong>해제</strong></div><div><i class="status-dot ok"></i><span>MCU fault</span><strong>없음</strong></div></div></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmManual">모터 활성화</button></div>`);
    $('#confirmManual').addEventListener('click', async () => {
      const api = requireCmd();
      if (!api) return;
      const button = $('#confirmManual');
      setButtonBusy(button, true, '인계 중…');
      try {
        // cmd_arbiter 의 동작권을 web 으로 가져온다. 이때부터 직접 rpm/조향각
        // 명령이 통과하고, /cmd_vel_mux 로는 0 twist 만 나간다.
        const result = await api.acquireManual();
        state.manual.enabled = true;
        state.systemState = 'MANUAL_CONTROL';
        closeModal();
        addLog('WARN', 'cmd_arbiter', `동작권 -> ${result.active_owner} (직접 rpm/조향각)`);
        renderManual(); renderGlobal();
        toast('수동주행이 활성화되었습니다',
          '버튼을 누르거나 W/A/S/D 를 누르고 있는 동안만 이동합니다. 스페이스 = E-STOP.',
          'success');
      } catch (error) {
        toast('수동주행을 시작할 수 없습니다', error.message,
          error.status === 409 ? 'warning' : 'error');
      } finally {
        setButtonBusy(button, false);
      }
    });
  }

  async function exitManual() {
    stopManualCommand();
    state.manual.heldKeys.clear();
    const api = cmd();
    // 화면 상태는 **먼저** 내린다. 반납 요청이 실패해도 화면이 '조작 가능'으로
    // 남아 있으면 안 된다 — 그 상태에서 누르면 409 만 받고 이유는 모른다.
    state.manual.enabled = false;
    if (state.systemState === 'MANUAL_CONTROL') state.systemState = 'IDLE';
    renderManual(); renderGlobal();
    try {
      const result = await api?.releaseManual();
      addLog('INFO', 'cmd_arbiter', `동작권 반납 -> ${result?.active_owner ?? 'auto'}`);
      toast('수동주행을 종료했습니다', '동작권을 자율로 반납했습니다.');
    } catch (error) {
      toast('동작권 반납 실패', `${error.message} — 로봇 쪽에서 확인하세요.`, 'error');
    }
  }

  /**
   * 속도 한계. command_manager 의 실제 파라미터를 받아 두고 쓴다.
   *
   * 예전에는 0.45 / -0.15 / 0.8 / 0.3 이 여기 하드코딩돼 있었다. 지금은 우연히
   * 값이 맞을 뿐이라, 로봇 쪽만 바꾸면 화면이 조용히 어긋난다. 서버가 주기 전
   * 기본값은 남겨 두되(초기 렌더용), 받으면 덮는다.
   */
  const limits = { max_linear_x: 0.45, min_linear_x: -0.15, max_linear_y: 0.30, max_angular_z: 0.8 };

  function applyLimits(next) {
    Object.assign(limits, next || {});
    $('#limitForward').textContent = `${limits.max_linear_x.toFixed(2)} m/s`;
    $('#limitReverse').textContent = `${limits.min_linear_x.toFixed(2)} m/s`;
    $('#limitLateral').textContent = `${limits.max_linear_y.toFixed(2)} m/s`;
    $('#limitAngular').textContent = `${limits.max_angular_z.toFixed(2)} rad/s`;
    if (typeof next?.cmd_timeout_sec === 'number') {
      $('#limitCmdTimeout').textContent = `${next.cmd_timeout_sec.toFixed(2)} s`;
    }
    if (typeof next?.max_steer_rate_deg_s === 'number') {
      const node = $('#monSteerRate');
      if (node) node.textContent = `${next.max_steer_rate_deg_s.toFixed(0)} °/s`;
    }
    // crab 가용 여부는 command_manager 의 auto_crab_enabled 가 정한다.
    // 예전에는 화면에 '비활성' 이 고정 문구로 박혀 있어서, 로봇 쪽에서 켜도
    // 화면은 계속 못 쓴다고 말했다.
    if (typeof next?.auto_crab_enabled === 'boolean') {
      applyCrabAvailability(next.auto_crab_enabled);
    }
    renderManual();
  }

  /** crab(게걸음) 버튼을 파라미터에 맞춰 열고 닫는다. */
  function applyCrabAvailability(enabled) {
    state.crabEnabled = enabled;
    $$('[data-mode="crab"], [data-command="crab_left"], [data-command="crab_right"]')
      .forEach((node) => {
        node.disabled = !enabled;
        node.title = enabled ? ''
          : 'command_manager 의 auto_crab_enabled 가 false 입니다 '
            + '(측위 안정성을 위해 기본 비활성)';
      });
    const note = $('#crabNote');
    if (note) {
      note.textContent = enabled
        ? '크랩 사용 가능'
        : '크랩 비활성 — auto_crab_enabled=false';
    }
  }

  /**
   * 라이다 점군의 출처를 화면에 반영한다 (/api/health 의 lidar_source).
   *
   * 재생본과 실측이 같은 토픽으로 흐르기 때문에 화면만 봐서는 구분이 안 된다.
   * 실제로 집에서 랩실 맵 재생본을 실시간 스캔으로 오인한 적이 있다. 발행 노드
   * 이름은 백엔드만 알 수 있어서(브리지의 connectionGraph 는 빈 값만 준다)
   * 여기서 받아 띄운다.
   */
  function setLidarSource(info) {
    const banner = $('#replayBanner');
    if (!banner) return;
    // null = 조회 실패. '재생 아님'과 섞지 않는다 — 모르면 배너를 건드리지 않는다.
    if (!info || info.replay === null || info.replay === undefined) return;

    const replay = info.replay === true;
    if (replay === state.lidarReplay) return;   // 상태가 그대로면 로그도 안 남긴다
    state.lidarReplay = replay;
    banner.hidden = !replay;
    if (replay) {
      const who = (info.publishers || []).join(', ') || '알 수 없음';
      addLog('WARN', 'alm_web_backend',
        `${info.topic} 이 재생본입니다 (발행: ${who}) — 센서 출력이 아닙니다`);
    } else {
      addLog('INFO', 'alm_web_backend',
        `${info.topic} 실측 발행 확인 (${(info.publishers || []).join(', ') || '없음'})`);
    }
  }

  /* ── 수동주행: rpm / 조향각 직접 조작 ────────────────────────────────
   *
   * 이 탭은 twist 를 쓰지 않는다. m/s 로 명령하면 실제로 몇 m/s 가 나가는지
   * 아무도 모르기 때문이다 — 변환이 base_control.yaml 의 ##CONFIRM## 상수
   * (wheel_radius_m, gear_ratio, ...)에 통째로 걸려 있고 그게 미확정이다.
   * rpm 과 각도는 STM32 가 받는 단위 그대로라 변환이 없다.
   *
   * 데드맨은 **누르고 있는 동안만** 명령을 보내는 것이다. 두 겹으로 걸린다:
   *   브라우저 -> 백엔드   갱신이 0.4 s 끊기면 백엔드가 스트림을 놓는다
   *   백엔드   -> arbiter  토픽이 0.5 s 끊기면 cmd_arbiter 가 HELD 로 세운다
   * 그래서 탭을 닫든 브라우저가 죽든 로봇은 선다.
   */
  const MANUAL_MODE_ID = { normal: 1, spin: 4, crab: 3 };
  const MANUAL_STREAM_MS = 100;          // 10 Hz. 백엔드 유지시간(0.4 s)의 1/4

  /** 지금 눌린 방향키가 뜻하는 (rpm, 조향각). */
  function manualTargets(command) {
    const manual = state.manual;
    const factor = manual.multiplier;
    const maxRpm = limits.max_rpm ?? 3000;
    const maxSteer = limits.max_steer_deg ?? 30;
    if (!command || command === 'stop') return { rpm: 0, steer: manual.steer };

    // 제자리 회전/크랩은 STM32 가 고정 자세를 쓰므로 조향각을 우리가 안 정한다.
    // 회전 방향은 rpm 의 부호로 표현된다 (uart_protocol.md v2).
    if (manual.mode === 'spin') {
      const rpm = maxRpm * factor * (command === 'left' ? 1 : command === 'right' ? -1 : 0);
      return { rpm, steer: 0 };
    }
    if (manual.mode === 'crab') {
      const rpm = maxRpm * factor * (command === 'forward' ? 1 : command === 'reverse' ? -1 : 0);
      return { rpm, steer: 0 };
    }
    // normal: 전/후진은 rpm, 좌/우는 조향각. 둘은 **독립**이다 —
    // twist 처럼 하나가 다른 하나를 정하지 않으므로 동시에 잡을 수 있다.
    if (command === 'forward') return { rpm: maxRpm * factor, steer: manual.steer };
    if (command === 'reverse') return { rpm: -maxRpm * factor, steer: manual.steer };
    if (command === 'left') return { rpm: manual.rpm, steer: -maxSteer * factor };
    if (command === 'right') return { rpm: manual.rpm, steer: maxSteer * factor };
    return { rpm: 0, steer: manual.steer };
  }

  function startManualCommand(command) {
    if (!state.manual.enabled || state.estop || !state.hasControl) return;
    const api = cmd();
    if (!api) return;
    state.manual.command = command;
    const target = manualTargets(command);
    state.manual.rpm = target.rpm;
    state.manual.steer = target.steer;
    if (command === 'stop') {
      stopManualCommand();
      api.manualStop().catch(() => {});
      return;
    }
    if (state.manual.streamTimer) return;      // 이미 스트리밍 중 (다른 키 추가)
    const send = async () => {
      const now = manualTargets(state.manual.command);
      state.manual.rpm = now.rpm;
      state.manual.steer = now.steer;
      updateManualTelemetry();
      try {
        await api.manualCommand(now.rpm, now.steer,
          MANUAL_MODE_ID[state.manual.mode] ?? 1);
      } catch (error) {
        // 스트리밍 중 실패는 조용히 멈춘다 — 토스트를 10 Hz 로 띄울 수는 없다.
        stopManualCommand();
        toast('수동 명령이 끊겼습니다', error.message, 'error');
      }
    };
    send();
    state.manual.streamTimer = setInterval(send, MANUAL_STREAM_MS);
    updateManualTelemetry();
  }

  function stopManualCommand(render = true) {
    clearInterval(state.manual.streamTimer);
    state.manual.streamTimer = null;
    state.manual.command = null;
    state.manual.rpm = 0;
    // 조향각은 남긴다 — 손을 뗐다고 바퀴가 스스로 정면으로 돌아가면
    // 그건 세우는 게 아니라 조타다 (cmd_arbiter._tick_web 과 같은 규약).
    if (render) updateManualTelemetry();
  }

  /** 키보드 데드맨. pointer 전용이면 마우스가 죽었을 때 명령이 안 끊긴다. */
  const MANUAL_KEYS = { w: 'forward', s: 'reverse', a: 'left', d: 'right' };
  function onManualKeyDown(event) {
    if (state.tab !== 'manual' || !state.manual.enabled) return;
    if (event.repeat) return;
    const key = String(event.key || '').toLowerCase();
    if (key === ' ') { event.preventDefault(); estopFromKeyboard(); return; }
    const command = MANUAL_KEYS[key];
    if (!command) return;
    event.preventDefault();
    state.manual.heldKeys.add(key);
    startManualCommand(command);
  }

  function onManualKeyUp(event) {
    const key = String(event.key || '').toLowerCase();
    if (!MANUAL_KEYS[key]) return;
    state.manual.heldKeys.delete(key);
    if (!state.manual.heldKeys.size) stopManualCommand();
    else startManualCommand(MANUAL_KEYS[[...state.manual.heldKeys][0]]);
  }

  function estopFromKeyboard() {
    stopManualCommand();
    $('#estopButton')?.click();
  }


  function updateManualTelemetry() {
    const manual = state.manual;
    const maxRpm = limits.max_rpm ?? 3000;
    const maxSteer = limits.max_steer_deg ?? 30;
    $('#cmdRpm').textContent = manual.rpm.toFixed(0);
    $('#cmdSteer').textContent = manual.steer.toFixed(1);
    $('#cmdRpmBar').style.width = `${clamp(Math.abs(manual.rpm) / maxRpm * 100, 0, 100)}%`;
    $('#cmdSteerBar').style.width = `${clamp(Math.abs(manual.steer) / maxSteer * 100, 0, 100)}%`;
    // 실제값은 /mcu/command 를 그대로 읽는다 (ingest 가 넣어준다).
    // 지어내지 않는 것이 요점이다 — 예전에는 여기서 명령값에 Math.random() 을
    // 곱해 "측정값" 을 만들었고, 로봇이 꺼져 있어도 바퀴 속도가 표시됐다.
    const actual = manual.actualRpm;
    $('#actualRpm').textContent = actual == null ? '—' : actual.toFixed(0);
    $('#actualRpmBar').style.width =
      `${actual == null ? 0 : clamp(Math.abs(actual) / maxRpm * 100, 0, 100)}%`;

    // 요청은 있는데 실제가 0 이면 십중팔구 모드 전환 dwell 이다.
    // 그 사실을 말해주지 않으면 조작자는 "밀었는데 안 간다" 로만 겪는다.
    const stalled = Math.abs(manual.rpm) > 1 && actual != null && Math.abs(actual) < 1;
    const notice = $('#manualDwellNotice');
    if (notice) {
      notice.classList.toggle('hidden', !stalled);
      notice.textContent = stalled
        ? '조향축이 자리 잡는 중입니다 — 모드 전환 dwell 동안 구동이 0 으로 유지됩니다. '
        + '바퀴가 굴러가면서 조향축이 스윕하면 의도와 다른 방향으로 갑니다.'
        : '';
    }
  }

  function renderMetrics() {
    const m = state.metrics;
    $('#sidebarCpu').textContent = `${Math.round(m.cpu)}%`; $('#sidebarCpuBar').style.width = `${m.cpu}%`;
    $('#mappingCpu').textContent = Math.round(m.cpu); $('#mappingRam').textContent = Math.round(m.ram); $('#mappingGpu').textContent = Math.round(m.gpu);
    $('#mappingTemp').textContent = `${m.temp.toFixed(1)}°C`;
    $('#cpuRing').style.setProperty('--value', `${m.cpu * 3.6}deg`);
    $('#ramRing').style.setProperty('--value', `${m.ram * 3.6}deg`);
    $('#gpuRing').style.setProperty('--value', `${m.gpu * 3.6}deg`);
    $('#monCpu').textContent = `${Math.round(m.cpu)}%`; $('#monCpuBar').style.width = `${m.cpu}%`;
    $('#monGpu').textContent = `${Math.round(m.gpu)}%`; $('#monGpuBar').style.width = `${m.gpu}%`;
    $('#monRam').textContent = `${(7.8 * m.ram / 100).toFixed(1)} / 7.8 GB`; $('#monRamBar').style.width = `${m.ram}%`;
    $('#monPower').textContent = `${m.power.toFixed(1)} W`;
    $('#tempCpu').textContent = `${m.temp.toFixed(1)}°C`; $('#tempGpu').textContent = `${(m.temp - 2.1).toFixed(1)}°C`;
    $('#tempSoc').textContent = `${(m.temp - 2.7).toFixed(1)}°C`; $('#tempTj').textContent = `${(m.temp + 1.8).toFixed(1)}°C`;
    $('#batteryVoltage').textContent = `${(21.0 + m.battery * 0.033).toFixed(1)} V`;
    $('#batteryCurrent').textContent = `${(1.2 + Math.random() * 0.8).toFixed(1)} A`;
    $('#pointsRate').textContent = `${Math.round(121000 + Math.random() * 6000).toLocaleString()} pts/s`;
    $('#netTx').textContent = `${(1.4 + Math.random() * 0.8).toFixed(1)} Mbps`; $('#netRx').textContent = `${(2.8 + Math.random() * 0.9).toFixed(1)} Mbps`;
  }

  function updateMetrics() {
    const activity = state.systemState === 'MAPPING' ? 14 : state.systemState === 'NAVIGATING' ? 8 : 0;
    state.metrics.cpu = clamp(31 + activity + (Math.random() - 0.5) * 8, 12, 86);
    state.metrics.gpu = clamp(25 + (state.tab === 'mapping' ? 8 : 0) + (Math.random() - 0.5) * 7, 5, 78);
    state.metrics.ram = clamp(51 + activity * 0.2 + (Math.random() - 0.5) * 3, 35, 86);
    state.metrics.temp = clamp(44.5 + activity * 0.17 + (Math.random() - 0.5) * 1.3, 38, 75);
    state.metrics.power = clamp(8.4 + activity * 0.09 + Math.random() * 0.8, 6, 14.5);
    state.metrics.battery = clamp(state.metrics.battery - 0.002, 0, 100);
    state.chart.push({ cpu: state.metrics.cpu, gpu: state.metrics.gpu, ram: state.metrics.ram });
    if (state.chart.length > 80) state.chart.shift();
    renderMetrics(); drawSystemChart();
  }

  function drawSystemChart() {
    const canvas = $('#systemChart');
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
    }
    const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width, h = rect.height; ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = 'rgba(255,255,255,.07)'; ctx.lineWidth = 1;
    for (let i = 1; i < 5; i += 1) { const y = h * i / 5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    const lines = [['cpu','#6FA8FF'],['gpu','#A88BFF'],['ram','#4ADE9B']];
    lines.forEach(([key, color]) => {
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
      state.chart.forEach((value, index) => {
        const x = state.chart.length <= 1 ? 0 : index / (state.chart.length - 1) * w;
        const y = h - value[key] / 100 * h;
        index ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }); ctx.stroke();
    });
  }

  function exportSnapshot() {
    const snapshot = {
      generated_at: new Date().toISOString(),
      system_state: state.systemState,
      pipeline: 'fpfh_teaser',
      active_map: state.activeMap,
      estop: state.estop,
      localization: state.localization,
      metrics: state.metrics,
      alarms: state.alarms,
    };
    const blob = new Blob([JSON.stringify(snapshot, null, 2)], { type: 'application/json' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob); link.download = `alm-system-snapshot-${Date.now()}.json`; link.click();
    URL.revokeObjectURL(link.href);
    toast('진단 스냅샷을 내보냈습니다', 'JSON 파일에 현재 상태와 알람이 포함되었습니다.', 'success');
  }

  function drawPointCloud() {
    const canvas = $('#pointCloudCanvas');
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
    }
    const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width, h = rect.height;
    const grad = ctx.createLinearGradient(0, 0, w, h); grad.addColorStop(0, '#101d31'); grad.addColorStop(1, '#07111f');
    ctx.fillStyle = grad; ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = 'rgba(86,126,179,.16)'; ctx.lineWidth = 1;
    for (let x = -w; x < w * 2; x += 42) { ctx.beginPath(); ctx.moveTo(w / 2, h * .43); ctx.lineTo(x, h); ctx.stroke(); }
    for (let i = 0; i < 12; i += 1) { const y = h * .43 + i * 28; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    const t = performance.now() * .00025;
    const cx = w * .5, cy = h * .49;
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(-.08);
    for (let i = 0; i < 1700; i += 1) {
      const angle = (i * 1.618 + t) % (Math.PI * 2);
      const radius = 45 + ((i * 37) % 300);
      const room = i % 5 === 0 ? 1 : .58;
      let x = Math.cos(angle) * radius * room;
      let y = Math.sin(angle) * radius * .42 * room;
      if (i % 3 === 0) x = Math.sign(x) * (85 + (i % 250));
      if (i % 7 === 0) y = Math.sign(y || 1) * (55 + (i % 90));
      ctx.fillStyle = i % 13 === 0 ? 'rgba(43,196,255,.95)' : `rgba(130,174,255,${.23 + (i % 5) * .08})`;
      ctx.fillRect(x, y, i % 13 === 0 ? 2 : 1.2, i % 13 === 0 ? 2 : 1.2);
    }
    ctx.strokeStyle = '#3fef89'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(-170, 90); ctx.quadraticCurveTo(-30, 45, 90, 75); ctx.stroke();
    ctx.fillStyle = '#fff'; ctx.fillRect(76, 63, 28, 17); ctx.fillStyle = '#2f6fff'; ctx.fillRect(83, 58, 15, 9);
    ctx.restore(); requestAnimationFrame(drawPointCloud);
  }

  function seedScanPoints() {
    const points = [];
    for (let i = 0; i < 88; i += 1) {
      const angle = i / 88 * Math.PI * 2;
      const radius = 82 + Math.sin(i * .81) * 18;
      points.push(`<circle cx="${370 + Math.cos(angle) * radius}" cy="${300 + Math.sin(angle) * radius}" r="1.7"/>`);
    }
    $('#scanLayer').innerHTML = points.join('');
  }

  function openSettings() {
    const api = cmd();
    if (api) {
      $('#backendUrl').value = api.base;
      // 토큰은 되보여주지 않는다. 있으면 있다고만 표시한다.
      $('#backendToken').value = '';
      $('#backendToken').placeholder = api.hasToken() ? '저장됨 (바꾸려면 새로 입력)' : '토큰을 입력하세요';
    }
    $('#settingsDrawer').classList.add('open');
    $('#drawerBackdrop').classList.remove('hidden');
  }
  function closeSettings() {
    $('#settingsDrawer').classList.remove('open');
    $('#drawerBackdrop').classList.add('hidden');
  }

  function saveSettings() {
    if (state.systemState !== 'IDLE') {
      toast('설정을 변경할 수 없습니다', '실행 중인 워크플로를 먼저 종료하세요.', 'warning'); return;
    }
    // 활성 맵 전환은 select 의 change 에서 즉시 서버로 나간다.
    // 여기서 저장하는 것은 접속 정보(백엔드 주소·토큰)뿐이다.
    const api = cmd();
    if (api) {
      const url = $('#backendUrl').value.trim();
      const token = $('#backendToken').value.trim();
      if (url && url !== api.base) api.setBaseUrl(url);
      if (token) api.setToken(token);
    }
    closeSettings();
    toast('설정을 저장했습니다', '접속 정보는 이 탭에서만 유지됩니다(sessionStorage).', 'success');
  }

  function bindEvents() {
    $$('.nav-item').forEach((button) => button.addEventListener('click', () => switchTab(button.dataset.tab)));
    $('#globalEstop').addEventListener('click', triggerEstop);
    $('#releaseEstop').addEventListener('click', requestEstopRelease);
    $('#controlRoleButton').addEventListener('click', toggleControl);
    $('#profilePill').addEventListener('click', openSettings);
    $('#mapPill').addEventListener('click', openMapManager);
    $('#modalBackdrop').addEventListener('click', (event) => { if (event.target === $('#modalBackdrop')) closeModal(); });

    $('#startMapping').addEventListener('click', startMapping);
    $('#stopMapping').addEventListener('click', stopMapping);
    $('#savePcd').addEventListener('click', savePcd);
    $('#openPcd2Pgm').addEventListener('click', openPcd2Pgm);
    $('#buildFpfhDb').addEventListener('click', openFpfhBuilder);
    $('#openMapManager').addEventListener('click', openMapManager);
    $('#newMapButton').addEventListener('click', openNewMapModal);
    $('#clearLogs').addEventListener('click', () => { state.logs = []; renderLogs(); });
    $('#logLevel').addEventListener('change', renderLogs);
    $('#reset3d').addEventListener('click', () => toast('3D 카메라를 초기화했습니다'));
    $('#topView3d').addEventListener('click', () => toast('탑다운 뷰로 전환했습니다'));
    $$('.viewport-toolbar .tool').forEach((button) => button.addEventListener('click', () => button.classList.toggle('active')));

    $('#addWaypointMode').addEventListener('click', toggleWaypointMode);
    // 웨이포인트는 '누른 자리 = 위치, 끈 방향 = 헤딩' 으로 만든다 (RViz 2D Goal Pose).
    // click 은 manualPose 목업 경로만 남는다 — mapClick 이 addWaypoint 를 건너뛴다.
    $('#navigationMap').addEventListener('click', mapClick);
    $('#navigationMap').addEventListener('pointerdown', beginGoalDrag);
    $('#navigationMap').addEventListener('pointermove', updateGoalDrag);
    $('#navigationMap').addEventListener('pointerup', endGoalDrag);
    // 포인터가 취소되면(다른 창으로 전환, 제스처 가로채기) 미리보기가 남지 않게 한다
    $('#navigationMap').addEventListener('pointercancel', () => {
      state.goalDrag = null;
      $('#goalPreviewLayer').innerHTML = '';
    });
    $('#navigationMap').addEventListener('mousemove', (event) => {
      const point = svgPoint(event); const map = pixelToMap(point.x, point.y);
      $('#mapCoordinates').textContent = `x ${map.x.toFixed(2)} · y ${map.y.toFixed(2)}`;
    });
    $('#zoomIn').addEventListener('click', () => { state.mapScale = clamp(state.mapScale + .12, .7, 1.8); $('#navigationMap').style.transform = `scale(${state.mapScale})`; });
    $('#zoomOut').addEventListener('click', () => { state.mapScale = clamp(state.mapScale - .12, .7, 1.8); $('#navigationMap').style.transform = `scale(${state.mapScale})`; });
    $('#fitMap').addEventListener('click', () => { state.mapScale = 1; $('#navigationMap').style.transform = ''; toast('지도를 화면에 맞췄습니다'); });
    $('#resetRotation').addEventListener('click', () => toast('지도 회전을 0°로 초기화했습니다'));
    $('#mapPanToggle').addEventListener('click', (event) => { event.currentTarget.classList.toggle('active'); toast('지도 패닝 모드를 전환했습니다'); });
    $$('[data-map-layer]').forEach((input) => input.addEventListener('change', () => $(`#${input.dataset.mapLayer}`).classList.toggle('hidden-layer', !input.checked)));
    $('#manualInitialPose').addEventListener('click', manualInitialPose);
    $('#autoLocalization').addEventListener('click', autoLocalization);
    $('#stopLocalizationBtn')?.addEventListener('click', stopLocalizationClicked);
    $('#relocalize').addEventListener('click', relocalize);

    // ── 정합 로그 ──
    $('#locLogSource')?.addEventListener('change', (event) => {
      state.locLogSource = event.target.value;
      state.locLogFollow = true;
      renderLocalizationLogs();
    });
    $('#clearLocLogs')?.addEventListener('click', () => {
      state.localizationLogs = [];
      renderLocalizationLogs();
    });
    $('#copyLocLogs')?.addEventListener('click', async () => {
      const rows = state.localizationLogs
        .filter((line) => (line.source || 'ros') === state.locLogSource)
        .map((line) => `${line.time} ${line.level || ''} ${line.node || ''} ${line.text || ''}`);
      if (!rows.length) { toast('복사할 로그가 없습니다', '', 'warning'); return; }
      try {
        await navigator.clipboard.writeText(rows.join('\n'));
        toast('로그를 복사했습니다', `${rows.length} 줄`, 'success');
      } catch {
        // https 가 아니면 clipboard API 가 없다. 로봇은 http 로 뜨므로 흔한 경우다.
        toast('클립보드를 쓸 수 없습니다', 'http 접속에서는 브라우저가 막습니다 — 직접 선택해 복사하세요.', 'warning');
      }
    });
    // 위로 올려 읽는 중이면 자동 스크롤을 멈춘다. 매핑 탭 로그창은 새 줄이
    // 올 때마다 무조건 맨 아래로 밀어서, 읽으려고 올리면 계속 튕겨 내려간다.
    $('#locLogWindow')?.addEventListener('scroll', (event) => {
      const box = event.currentTarget;
      const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 24;
      if (atBottom === state.locLogFollow) return;
      state.locLogFollow = atBottom;
      $('#locLogFollow')?.classList.toggle('hidden', atBottom);
    });
    $('#locLogFollow')?.addEventListener('click', () => {
      state.locLogFollow = true;
      $('#locLogFollow').classList.add('hidden');
      const box = $('#locLogWindow');
      if (box) box.scrollTop = box.scrollHeight;
    });
    $('#startNavigation').addEventListener('click', startNavigation);
    $('#pauseNavigation').addEventListener('click', pauseNavigation);
    $('#cancelNavigation').addEventListener('click', cancelNavigation);
    $('#clearAlarms').addEventListener('click', () => { state.alarms = []; renderAlarms(); });
    $('#saveWaypointSet').addEventListener('click', () => state.waypoints.length ? toast('웨이포인트 세트를 저장했습니다', `${state.activeMap} · ${state.waypoints.length} points`, 'success') : toast('저장할 웨이포인트가 없습니다', '', 'warning'));
    $('#loadWaypointSet').addEventListener('click', () => {
      state.waypoints = [
        { id: 'saved-1', label: '출발', x: -1.6, y: 0.2, yaw: 0, px: 370, py: 300, tolerance: .25 },
        { id: 'saved-2', label: '검사 구역', x: 1.8, y: -0.8, yaw: 90, px: 540, py: 350, tolerance: .25 },
        { id: 'saved-3', label: '복귀', x: 5.1, y: -1.6, yaw: 180, px: 705, py: 390, tolerance: .25 },
      ]; renderWaypoints(); toast('저장된 세트를 불러왔습니다', '순찰 A · 3 points', 'success');
    });
    $$('#navDriveModes button:not(:disabled)').forEach((button) => button.addEventListener('click', () => {
      state.nav.mode = button.dataset.mode; $$('#navDriveModes button').forEach((item) => item.classList.toggle('active', item === button)); renderNavigation();
    }));

    // 키보드 데드맨 (#14). pointer 전용이면 마우스가 죽었을 때 안 끊긴다.
    window.addEventListener('keydown', onManualKeyDown);
    window.addEventListener('keyup', onManualKeyUp);
    // 창이 포커스를 잃으면 keyup 이 안 온다 — 키가 눌린 채로 남는다.
    window.addEventListener('blur', () => {
      state.manual.heldKeys.clear();
      stopManualCommand();
    });
    $('#enterManual').addEventListener('click', enterManual);
    $('#exitManual').addEventListener('click', exitManual);
    $$('#manualModeSelector button').forEach((button) => button.addEventListener('click', () => {
      stopManualCommand();
      if (button.dataset.mode === 'crab') {
        state.manual.mode = 'crab'; renderManual(); toast('Crab 모드는 현재 비활성입니다', 'auto_crab_enabled=false', 'warning'); return;
      }
      state.manual.mode = button.dataset.mode; renderManual(); toast('주행 모드를 변경했습니다', button.dataset.mode);
    }));
    $$('#speedMultiplier button').forEach((button) => button.addEventListener('click', () => { state.manual.multiplier = Number(button.dataset.value) / 100; renderManual(); }));
    $$('.drive-button').forEach((button) => {
      button.addEventListener('pointerdown', (event) => { event.preventDefault(); button.setPointerCapture(event.pointerId); startManualCommand(button.dataset.command); });
      button.addEventListener('pointerup', () => stopManualCommand());
      button.addEventListener('pointercancel', () => stopManualCommand());
      button.addEventListener('pointerleave', (event) => { if (event.buttons) stopManualCommand(); });
    });
    document.addEventListener('visibilitychange', () => { if (document.hidden) stopManualCommand(); });

    $('#exportSnapshot').addEventListener('click', exportSnapshot);
    $('#refreshProcesses').addEventListener('click', () => { toast('ROS 그래프와 PID를 다시 조회했습니다', '14개 프로세스 정상', 'success'); });

    $('#openSettings').addEventListener('click', openSettings);
    $('#closeSettings').addEventListener('click', closeSettings);
    $('#drawerBackdrop').addEventListener('click', closeSettings);
    $('#settingsMapSelect').addEventListener('change', (event) => changeActiveMap(event.target.value));
    $('#testConnection').addEventListener('click', async (event) => {
      const api = cmd();
      setButtonBusy(event.currentTarget, true, '테스트 중…');
      const started = performance.now();
      try {
        const health = await api.health();
        const ms = Math.round(performance.now() - started);
        toast('연결 테스트 성공',
          `백엔드 ${ms} ms · 활성 맵 ${health.active_map} · /map_save ${health.map_save_available ? '가능' : '미기동'}`,
          'success');
      } catch (error) {
        toast('연결 테스트 실패', error.message, 'error');
      } finally {
        setButtonBusy(event.currentTarget, false);
      }
    });
    $('#saveSettings').addEventListener('click', saveSettings);
    window.addEventListener('resize', drawSystemChart);
  }

  function init() {
    resetMappingSteps();
    renderMapOptions();
    seedScanPoints();
    const live = isLive();
    if (!live) {
      // 목업 시드. 라이브에서는 /rosout 과 /mcu/state 가 이 자리를 채운다.
      state.alarms = [
        { type: 'info', title: '시스템 준비 완료', detail: 'Bridge · Backend · MCU 연결 정상', time: nowTime() },
        { type: 'warning', title: 'MID-360 상태 패킷 미확정', detail: '온도 항목은 status_available=false', time: nowTime() },
      ];
      state.logs = [
        { time: nowTime(), level: 'INFO', node: 'alm_web_backend', text: 'Backend ready. Restored 4 managed processes.' },
        { time: nowTime(), level: 'INFO', node: 'foxglove_bridge', text: 'WebSocket connected with topic allowlist.' },
        { time: nowTime(), level: 'INFO', node: 'command_manager', text: '안전 게이팅 활성 (cmd_timeout 0.5 s · E-STOP 래치).' },
        { time: nowTime(), level: 'WARN', node: 'lidar_health', text: 'Device status packet parser unavailable; basic health metrics only.' },
      ];
      for (let i = 0; i < 50; i += 1) state.chart.push({ cpu: 28 + Math.random() * 9, gpu: 21 + Math.random() * 10, ram: 48 + Math.random() * 5 });
    }
    bindEvents();
    renderGlobal(); renderMappingSteps(); renderLogs(); renderWaypoints(); renderLocalization(); renderNavigation(); renderAlarms(); renderManual(); renderMetrics();
    requestAnimationFrame(drawSystemChart);

    if (!live) {
      // 라이브에서는 renderer3d.js 가 #pointCloudCanvas 를 WebGL 컨텍스트로
      // 가져가고, 수치는 구독 콜백이 넣는다. 목업 루프가 함께 돌면 캔버스
      // 컨텍스트가 충돌하고 실데이터를 매초 덮어쓴다.
      requestAnimationFrame(drawPointCloud);
      setInterval(updateMetrics, 1000);
    }
  }

  document.addEventListener('DOMContentLoaded', init);

  // ── 라이브 연동 접점 ────────────────────────────────────────────────
  // app.js 는 IIFE 라 외부에서 상태를 넣을 방법이 없다. 구독 콜백(src/ingest.js)이
  // state 를 갱신하고 기존 render*() 를 다시 부를 수 있도록 최소한만 노출한다.
  // 여기 없는 것은 의도적으로 닫아둔 것 — 특히 명령 계열 함수는 읽기 전용
  // 단계에서 브라우저가 호출할 일이 없어야 한다.
  window.ALM = {
    state,
    isLive,
    esc,
    renderGlobal, renderMetrics, renderManual, renderLocalization,
    renderNavigation, renderAlarms, renderLogs, drawSystemChart,
    // 축척이 바뀌면 핀 위치를 다시 계산해야 한다 (핀은 미터로 저장돼 있다)
    renderWaypoints,
    // 수동주행 '실제 rpm' 은 /mcu/command 에서 온다 (ingest 가 넣고 다시 그린다)
    updateManualTelemetry,
    // /alm/map_inventory 가 이미 만들어진 자산의 단계를 done 으로 접을 때 쓴다
    renderMappingSteps, renderMapOptions,
    // 서버가 본 slam 프로세스 상태 변화 (다른 경로로 시작·종료된 매핑)
    onSlamRunningChange,
    // 측위: 슬롯 상태 변화 / /rosout 진행 로그 / /icp_result 수렴
    onLocalizationRunningChange, addLocalizationLog, onLocalizationConverged,
    renderLocalizationLogs,
    // 자율주행: /api/navigation 폴링 시작(main.js 가 토큰 확인 후 부른다)
    startNavPoll, applyNavStatus,
    addLog, addAlarm, toast, openSettings,
    // command_manager 의 실제 속도 한계를 받아 하드코딩을 덮는다
    applyLimits,
    // 점군이 실측인지 재생본인지 (health 폴링이 3초마다 넣어준다)
    setLidarSource,
  };
})();
