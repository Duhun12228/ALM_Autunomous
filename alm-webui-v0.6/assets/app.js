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
    localization: { state: 'IDLE', frame: 0, fitness: null, pose: null },
    waypoints: [],
    addWaypoint: false,
    manualPose: false,
    mapScale: 1,
    nav: { state: 'IDLE', progress: 0, current: 0, distance: 0, eta: 0, timer: null, mode: 'auto' },
    alarms: [],
    manual: { enabled: false, mode: 'normal', multiplier: 0.5, command: null, timer: null, cmd: { x: 0, y: 0, z: 0 } },
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

  function switchTab(tab) {
    if (!TAB_META[tab]) return;
    if (tab === 'navigation' && state.systemState === 'MAPPING') {
      toast('탭이 잠겨 있습니다', 'SLAM 매핑 중에는 자율주행을 시작할 수 없습니다.', 'warning');
    }
    if (tab === 'manual' && state.systemState === 'MAPPING') {
      toast('웹 수동주행 잠금', '매핑 중 차량 이동은 외부 조이스틱으로만 수행합니다.', 'warning');
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
      : 'E-STOP 해제 · 감독 하트비트 정상 · 활성 fault 없음';
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

  function buildFpfhDb() {
    if (!hasCloudAsset()) {
      toast('PCD가 필요합니다', '먼저 3D 맵을 저장하세요.', 'warning');
      return;
    }
    return runAssetJob({
      button: $('#buildFpfhDb'),
      stepKey: 'fpfh_db',
      label: 'fpfh_map_builder',
      run: (api) => api.runFpfh({ map: state.activeMap }),
      done: 'FPFH 측위 DB를 생성했습니다',
    });
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

  function renderWaypoints() {
    $('#waypointCount').textContent = state.waypoints.length;
    $('#waypointEmpty').classList.toggle('hidden', state.waypoints.length > 0);
    $('#waypointList').innerHTML = state.waypoints.map((point, index) => `
      <div class="waypoint-item" draggable="true" data-id="${esc(point.id)}"><span class="waypoint-number">${index + 1}</span><div><strong>${esc(point.label)}</strong><small>x ${point.x.toFixed(2)} · y ${point.y.toFixed(2)} · yaw ${esc(point.yaw)}°</small></div><button data-remove-waypoint="${esc(point.id)}">×</button></div>`).join('');
    $$('[data-remove-waypoint]').forEach((button) => button.addEventListener('click', () => {
      state.waypoints = state.waypoints.filter((point) => String(point.id) !== button.dataset.removeWaypoint);
      renderWaypoints();
    }));
    $('#waypointLayer').innerHTML = state.waypoints.map((point, index) => `
      <g transform="translate(${Number(point.px) || 0} ${Number(point.py) || 0})"><circle r="17" fill="#2f6fff" stroke="#fff" stroke-width="5"/><text y="4" text-anchor="middle" fill="#fff" font-size="11" font-weight="800">${index + 1}</text><path d="M0-27L5-19H-5Z" fill="#2f6fff"/></g>`).join('');
  }

  function toggleWaypointMode() {
    if (!canOperate()) return;
    state.addWaypoint = !state.addWaypoint;
    state.manualPose = false;
    $('#addWaypointMode').classList.toggle('active', state.addWaypoint);
    $('#mapModeHint').textContent = state.addWaypoint ? '맵을 클릭해 웨이포인트를 추가하세요.' : '맵을 탐색하거나 웨이포인트를 추가하세요.';
  }

  function mapClick(event) {
    const point = svgPoint(event);
    if (state.addWaypoint) {
      const map = pixelToMap(point.x, point.y);
      state.waypoints.push({
        id: `${Date.now()}-${Math.random()}`,
        label: `Waypoint ${state.waypoints.length + 1}`,
        x: map.x, y: map.y, yaw: 0,
        px: point.x, py: point.y,
        tolerance: Number($('#defaultTolerance').value),
      });
      renderWaypoints();
      toast('웨이포인트를 추가했습니다', `x ${map.x.toFixed(2)} · y ${map.y.toFixed(2)}`, 'success');
      return;
    }
    if (state.manualPose) {
      const map = pixelToMap(point.x, point.y);
      state.localization.pose = { x: map.x, y: map.y, yaw: 0 };
      state.localization.state = 'WAITING_ICP';
      state.manualPose = false;
      renderLocalization();
      setTimeout(() => completeLocalization('manual'), 1400);
      return;
    }
  }

  function renderLocalization() {
    const loc = state.localization;
    const chip = $('#localizationChip');
    chip.className = 'state-chip';
    $('#localizationProgress').classList.toggle('hidden', !['ACCUMULATING', 'MATCHING', 'WAITING_ICP'].includes(loc.state));
    if (loc.state === 'IDLE') {
      chip.classList.add('idle'); chip.textContent = '대기';
      $('#localizationTitle').textContent = '초기위치가 필요합니다';
      $('#localizationDetail').textContent = 'FPFH + TEASER++ 자동 초기화 가능';
    } else if (loc.state === 'ACCUMULATING') {
      chip.classList.add('running'); chip.textContent = '누적 중';
      $('#localizationTitle').textContent = '라이다 프레임 누적 중';
      $('#localizationDetail').textContent = '로봇을 정지 상태로 유지하세요.';
      $('#localizationProgressText').textContent = `프레임 ${loc.frame} / 10 누적`;
      $('#localizationProgressBar').style.width = `${loc.frame * 10}%`;
    } else if (loc.state === 'MATCHING') {
      chip.classList.add('running'); chip.textContent = '매칭';
      $('#localizationTitle').textContent = 'FPFH 대응점 탐색';
      $('#localizationDetail').textContent = 'mutual + ratio 검증을 통과한 대응점을 모으고 있습니다.';
      $('#localizationProgressText').textContent = 'FPFH correspondence matching';
      $('#localizationProgressBar').style.width = '62%';
    } else if (loc.state === 'WAITING_ICP') {
      chip.classList.add('warning'); chip.textContent = 'ICP';
      $('#localizationTitle').textContent = '후보 2 / 5 시도 중';
      $('#localizationDetail').textContent = 'ICP 수렴을 최대 12초 동안 확인합니다.';
      $('#localizationProgressText').textContent = 'ICP elapsed 4.2 / 12.0 s';
      $('#localizationProgressBar').style.width = '78%';
    } else if (loc.state === 'CONVERGED') {
      chip.classList.add('success'); chip.textContent = '수렴';
      $('#localizationTitle').textContent = '초기위치가 확정되었습니다';
      $('#localizationDetail').textContent = 'map → odom 변환이 안정적으로 연결되었습니다.';
    } else {
      chip.classList.add('warning'); chip.textContent = '실패';
      $('#localizationTitle').textContent = '측위에 실패했습니다';
      $('#localizationDetail').textContent = '로봇 위치를 이동한 뒤 다시 시도하세요.';
    }
    $('#fitnessText').textContent = loc.fitness == null ? '—' : loc.fitness.toFixed(3);
    $('#poseText').textContent = loc.pose ? `${loc.pose.x.toFixed(2)}, ${loc.pose.y.toFixed(2)}, ${loc.pose.yaw}°` : '미확정';
    $('#confidenceText').textContent = loc.state === 'CONVERGED' ? '높음' : '—';
    $('#localizationModeText').textContent = 'FPFH + TEASER++';
  }

  async function autoLocalization() {
    if (!canOperate()) return;
    if (state.systemState !== 'IDLE') {
      toast('측위를 시작할 수 없습니다', '현재 작업을 먼저 종료하세요.', 'warning');
      return;
    }
    state.systemState = 'LOCALIZING';
    state.localization = { state: 'ACCUMULATING', frame: 0, fitness: null, pose: null };
    $('#candidateLayer').classList.remove('hidden-layer');
    renderLocalization(); renderGlobal();
    addLog('INFO', 'teaser_fpfh_localizer', 'Relocalization requested. Accumulating 10 frames.');
    for (let frame = 1; frame <= 10; frame += 1) {
      await wait(180);
      if (state.localization.state !== 'ACCUMULATING') return;
      state.localization.frame = frame; renderLocalization();
    }
    state.localization.state = 'MATCHING'; renderLocalization(); await wait(900);
    state.localization.state = 'WAITING_ICP';
    $('#candidateLayer').innerHTML = [[280,220],[410,265],[550,350],[620,235],[700,420]].map(([x,y], index) => `<circle cx="${x}" cy="${y}" r="${12-index}" fill="#8c61ff" opacity="${0.85-index*0.1}"/>`).join('');
    renderLocalization(); await wait(1300);
    completeLocalization('fpfh_teaser');
  }

  function completeLocalization(mode) {
    state.localization.state = 'CONVERGED';
    state.localization.fitness = mode === 'manual' ? 0.164 : 0.087;
    state.localization.pose ||= { x: -1.6, y: 0.2, yaw: 12 };
    state.systemState = 'IDLE';
    $('#candidateLayer').classList.add('hidden-layer');
    renderLocalization(); renderGlobal();
    addAlarm('info', '초기위치 확정', `ICP fitness ${state.localization.fitness.toFixed(3)}`);
    addLog('INFO', 'teaser_fpfh_localizer', `Localization converged. mode=${mode} fitness=${state.localization.fitness.toFixed(3)}`);
    toast('초기위치가 확정되었습니다', `Fitness ${state.localization.fitness.toFixed(3)}`, 'success');
  }

  function manualInitialPose() {
    if (!canOperate()) return;
    state.manualPose = true; state.addWaypoint = false;
    $('#addWaypointMode').classList.remove('active');
    $('#mapModeHint').textContent = '맵을 클릭해 초기 위치를 지정하세요. 방향은 0°로 모의 적용됩니다.';
    toast('수동 초기위치 지정 모드', '맵에서 로봇 위치를 클릭하세요.');
  }

  function relocalize() {
    if (state.nav.state === 'RUNNING') {
      toast('주행 중 재측위할 수 없습니다', '주행을 중단하거나 일시정지한 뒤 시도하세요.', 'warning');
      return;
    }
    state.localization = { state: 'IDLE', frame: 0, fitness: null, pose: null };
    renderLocalization();
    autoLocalization();
  }

  function renderNavigation() {
    const nav = state.nav;
    const chip = $('#navStateChip');
    chip.className = 'state-chip';
    chip.textContent = nav.state;
    chip.classList.add(nav.state === 'RUNNING' ? 'running' : nav.state === 'PAUSED' ? 'warning' : nav.state === 'COMPLETED' ? 'success' : 'idle');
    $('#missionPercent').textContent = `${Math.round(nav.progress)}%`;
    $('#missionProgressRing').style.setProperty('--progress', `${nav.progress * 3.6}deg`);
    $('#currentGoalText').textContent = state.waypoints.length && nav.state !== 'IDLE' ? `${Math.min(nav.current + 1, state.waypoints.length)} / ${state.waypoints.length}` : '—';
    $('#remainingDistance').textContent = nav.state === 'IDLE' ? '—' : `${Math.max(0, nav.distance).toFixed(1)} m`;
    $('#etaText').textContent = nav.state === 'IDLE' ? '—' : `${Math.max(0, Math.round(nav.eta))} s`;
    const actual = nav.mode === 'auto' ? (nav.progress % 30 > 24 ? 'spin' : 'normal') : nav.mode;
    $('#navDriveMode').textContent = `${nav.mode} / ${actual}`;
    $('#startNavigation').disabled = nav.state === 'RUNNING' || nav.state === 'PAUSED';
    $('#pauseNavigation').disabled = !['RUNNING', 'PAUSED'].includes(nav.state);
    $('#pauseNavigation').textContent = nav.state === 'PAUSED' ? '재개' : '일시정지';
    $('#cancelNavigation').disabled = !['RUNNING', 'PAUSED'].includes(nav.state);
  }

  function startNavigation() {
    if (!canOperate()) return;
    if (state.systemState !== 'IDLE') {
      toast('자율주행을 시작할 수 없습니다', `현재 상태: ${state.systemState}`, 'warning'); return;
    }
    if (state.localization.state !== 'CONVERGED') {
      toast('초기위치가 필요합니다', '측위를 먼저 완료하세요.', 'warning'); return;
    }
    if (!state.waypoints.length) {
      toast('웨이포인트가 없습니다', '맵에서 하나 이상의 목표를 추가하세요.', 'warning'); return;
    }
    state.systemState = 'NAVIGATING';
    state.nav.state = 'RUNNING'; state.nav.progress = 0; state.nav.current = 0;
    state.nav.distance = state.waypoints.length * 7.5; state.nav.eta = state.nav.distance / 0.35;
    addAlarm('info', '자율주행 시작', `${state.waypoints.length}개 웨이포인트`);
    addLog('INFO', 'nav2', `FollowWaypoints accepted. poses=${state.waypoints.length}`);
    state.nav.timer = setInterval(() => {
      if (state.nav.state !== 'RUNNING') return;
      state.nav.progress = clamp(state.nav.progress + 0.8 + Math.random() * 1.1, 0, 100);
      state.nav.distance = Math.max(0, state.nav.distance * (1 - state.nav.progress / 7500));
      state.nav.eta = state.nav.distance / 0.35;
      state.nav.current = Math.min(state.waypoints.length - 1, Math.floor((state.nav.progress / 100) * state.waypoints.length));
      if (Math.random() < 0.02) addAlarm('warning', '경로 재계획', '장애물로 인해 글로벌 경로가 갱신되었습니다.');
      if (state.nav.progress >= 100) completeNavigation();
      renderNavigation();
    }, 350);
    renderNavigation(); renderGlobal();
    toast('자율주행을 시작했습니다', '감독 하트비트가 5 Hz로 유지됩니다.', 'success');
  }

  function pauseNavigation() {
    if (state.nav.state === 'RUNNING') {
      state.nav.state = 'PAUSED';
      addLog('INFO', 'alm_web_backend', 'Mission paused. Nav2 goal canceled; remaining waypoints retained.');
      toast('주행을 일시정지했습니다', '남은 웨이포인트를 유지합니다.', 'warning');
    } else if (state.nav.state === 'PAUSED') {
      state.nav.state = 'RUNNING';
      addLog('INFO', 'alm_web_backend', 'Mission resumed from remaining waypoint list.');
      toast('주행을 재개했습니다', '남은 경로로 새 목표를 전송했습니다.', 'success');
    }
    renderNavigation();
  }

  function cancelNavigation(showToast = true) {
    clearInterval(state.nav.timer); state.nav.timer = null;
    if (['RUNNING', 'PAUSED'].includes(state.nav.state)) addLog('WARN', 'nav2', 'Goal canceled and stop command asserted.');
    state.nav.state = 'IDLE'; state.nav.progress = 0; state.nav.current = 0; state.nav.distance = 0; state.nav.eta = 0;
    if (state.systemState === 'NAVIGATING') state.systemState = 'IDLE';
    renderNavigation(); renderGlobal();
    if (showToast) toast('자율주행을 중단했습니다', '목표 취소 후 정지 명령을 전송했습니다.', 'warning');
  }

  function completeNavigation() {
    clearInterval(state.nav.timer); state.nav.timer = null;
    state.nav.state = 'COMPLETED'; state.nav.progress = 100; state.nav.distance = 0; state.nav.eta = 0;
    state.systemState = 'IDLE';
    addAlarm('info', '미션 완료', '모든 웨이포인트에 도달했습니다.');
    addLog('INFO', 'nav2', 'FollowWaypoints completed successfully.');
    renderNavigation(); renderGlobal();
    toast('주행 미션이 완료되었습니다', `${state.waypoints.length}개 목표 도달`, 'success');
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
    $('#confirmManual').addEventListener('click', () => {
      state.manual.enabled = true; state.systemState = 'MANUAL_CONTROL'; closeModal();
      addLog('INFO', 'command_gateway', 'Manual control session armed. cmd_timeout=0.5s.');
      renderManual(); renderGlobal();
      toast('수동주행이 활성화되었습니다', '데드맨 버튼을 누르는 동안만 이동합니다.', 'success');
    });
  }

  function exitManual() {
    stopManualCommand();
    state.manual.enabled = false;
    if (state.systemState === 'MANUAL_CONTROL') state.systemState = 'IDLE';
    addLog('INFO', 'command_gateway', 'Manual control session disarmed.');
    renderManual(); renderGlobal();
    toast('수동주행을 종료했습니다', '모터 명령 출력을 비활성화했습니다.');
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
    renderManual();
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

  function commandFor(name) {
    const factor = state.manual.multiplier;
    const mode = state.manual.mode;
    if (name === 'stop') return { x: 0, y: 0, z: 0 };
    const wz = limits.max_angular_z;
    if (mode === 'spin') return { x: 0, y: 0, z: name === 'left' ? wz * factor : name === 'right' ? -wz * factor : 0 };
    if (mode === 'normal' || mode === 'auto') {
      if (name === 'forward') return { x: limits.max_linear_x * factor, y: 0, z: 0 };
      if (name === 'reverse') return { x: limits.min_linear_x * factor, y: 0, z: 0 };
      if (name === 'left') return { x: 0, y: 0, z: wz * factor };
      if (name === 'right') return { x: 0, y: 0, z: -wz * factor };
    }
    return { x: 0, y: 0, z: 0 };
  }

  function startManualCommand(command) {
    if (!state.manual.enabled || state.estop || !state.hasControl) return;
    stopManualCommand(false);
    state.manual.command = command;
    state.manual.cmd = commandFor(command);
    updateManualTelemetry();
    state.manual.timer = setInterval(() => {
      state.manual.cmd = commandFor(command);
      updateManualTelemetry();
    }, 50);
  }

  function stopManualCommand(render = true) {
    clearInterval(state.manual.timer); state.manual.timer = null;
    state.manual.command = null; state.manual.cmd = { x: 0, y: 0, z: 0 };
    if (render) updateManualTelemetry();
  }

  function updateManualTelemetry() {
    const cmd = state.manual.cmd;
    $('#cmdX').textContent = cmd.x.toFixed(2); $('#cmdY').textContent = cmd.y.toFixed(2); $('#cmdZ').textContent = cmd.z.toFixed(2);
    $('#cmdXBar').style.width = `${Math.abs(cmd.x) / 0.45 * 100}%`;
    $('#cmdYBar').style.width = `${Math.abs(cmd.y) / 0.3 * 100}%`;
    $('#cmdZBar').style.width = `${Math.abs(cmd.z) / 0.8 * 100}%`;
    const measuredX = cmd.x * (cmd.x ? 0.94 + Math.random() * 0.04 : 0);
    const measuredZ = cmd.z * (cmd.z ? 0.92 + Math.random() * 0.06 : 0);
    $('#measuredX').textContent = measuredX.toFixed(2); $('#measuredZ').textContent = measuredZ.toFixed(2);
    $('#measuredXBar').style.width = `${Math.abs(measuredX) / 0.45 * 100}%`;
    $('#measuredZBar').style.width = `${Math.abs(measuredZ) / 0.8 * 100}%`;
    const wheel = measuredX || Math.abs(measuredZ) * 0.31;
    ['speedFL','speedFR','speedRL','speedRR'].forEach((id, index) => {
      const offset = wheel ? (index % 2 ? -0.006 : 0.006) : 0;
      $(`#${id}`).textContent = (wheel + offset).toFixed(2);
    });
    const angle = cmd.z ? (cmd.z > 0 ? -32 : 32) : 0;
    ['wheelFL','wheelFR','wheelRL','wheelRR'].forEach((id, index) => {
      const sign = index < 2 ? 1 : -1;
      $(`#${id}`).style.transform = `rotate(${angle * sign}deg)`;
    });
    $('#frontSteer').textContent = `${angle.toFixed(0)}°`;
    $('#rearSteer').textContent = `${(-angle).toFixed(0)}°`;
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
    $('#buildFpfhDb').addEventListener('click', buildFpfhDb);
    $('#openMapManager').addEventListener('click', openMapManager);
    $('#newMapButton').addEventListener('click', openNewMapModal);
    $('#clearLogs').addEventListener('click', () => { state.logs = []; renderLogs(); });
    $('#logLevel').addEventListener('change', renderLogs);
    $('#reset3d').addEventListener('click', () => toast('3D 카메라를 초기화했습니다'));
    $('#topView3d').addEventListener('click', () => toast('탑다운 뷰로 전환했습니다'));
    $$('.viewport-toolbar .tool').forEach((button) => button.addEventListener('click', () => button.classList.toggle('active')));

    $('#addWaypointMode').addEventListener('click', toggleWaypointMode);
    $('#navigationMap').addEventListener('click', mapClick);
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
    $('#relocalize').addEventListener('click', relocalize);
    $('#startNavigation').addEventListener('click', startNavigation);
    $('#pauseNavigation').addEventListener('click', pauseNavigation);
    $('#cancelNavigation').addEventListener('click', () => cancelNavigation(true));
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
    window.addEventListener('blur', () => stopManualCommand());
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
        { time: nowTime(), level: 'INFO', node: 'safety_supervisor', text: 'Safety heartbeat armed at 5 Hz.' },
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
    // /alm/map_inventory 가 이미 만들어진 자산의 단계를 done 으로 접을 때 쓴다
    renderMappingSteps, renderMapOptions,
    // 서버가 본 slam 프로세스 상태 변화 (다른 경로로 시작·종료된 매핑)
    onSlamRunningChange,
    addLog, addAlarm, toast, openSettings,
    // command_manager 의 실제 속도 한계를 받아 하드코딩을 덮는다
    applyLimits,
    // 점군이 실측인지 재생본인지 (health 폴링이 3초마다 넣어준다)
    setLidarSource,
  };
})();
