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

  const PROFILE_INFO = {
    A: '수동 초기위치 · Scan Context DB 불필요 (PCD만 사용)',
    B: 'SC 자동 측위 · 매핑 후 Scan Context DB 생성 필요 (PCD + SC DB)',
    C: 'SC-LIO-SAM · GTSAM 포즈 그래프 최적화 (PCD + SC DB + GTSAM)',
  };

  const state = {
    tab: 'mapping',
    profile: 'B',
    activeMap: 'lab_main',
    mapRevision: 3,
    maps: [
      { name: 'lab_main', revision: 3, hash: '8fd2', date: '2026-07-23 09:14', hasPcd: true, hasPgm: true, hasScDb: true },
      { name: 'office_floor_1', revision: 2, hash: 'c14a', date: '2026-07-21 15:40', hasPcd: true, hasPgm: true, hasScDb: true },
      { name: 'outdoor_loop', revision: 1, hash: '0a77', date: '2026-07-18 13:02', hasPcd: true, hasPgm: true, hasScDb: false },
    ],
    estop: false,
    hasControl: true,
    systemState: 'IDLE',
    mappingElapsed: 0,
    mappingTimer: null,
    mappingSaved: false,
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
    ['sc_db', 'Scan Context DB 생성', '방식 B/C 전용'],
  ];

  function resetMappingSteps() {
    state.mappingSteps = STEP_BASE.map(([key, title, detail]) => ({
      key, title, detail,
      status: key === 'sc_db' && state.profile === 'A' ? 'skipped' : 'pending',
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

  function setButtonBusy(button, busy, label) {
    if (!button) return;
    if (busy) {
      button.dataset.original = button.textContent;
      button.textContent = label || '처리 중…';
      button.disabled = true;
    } else {
      button.textContent = button.dataset.original || button.textContent;
      button.disabled = false;
    }
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
    $('#profileText').textContent = state.profile;
    $('#activeMapText').textContent = state.activeMap;
    $('#mappingActiveMapText').textContent = state.activeMap;
    $('#settingsMapSelect').value = state.activeMap;
    $('#mappingProfile').value = state.profile;
    $('#profileDesc').textContent = PROFILE_INFO[state.profile];
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

  function triggerEstop() {
    state.estop = true;
    stopManualCommand();
    cancelNavigation(false);
    addAlarm('critical', 'E-STOP 활성화', '사용자가 전역 비상정지를 작동했습니다.');
    addLog('ERROR', 'safety_supervisor', 'E-STOP request latched. Motion commands blocked.');
    renderGlobal();
    renderManual();
    toast('비상정지가 활성화되었습니다', '모든 주행 명령이 차단되었습니다.', 'error');
  }

  function requestEstopRelease() {
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">SAFETY CONFIRMATION</p><h2>E-STOP 해제</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">차량 주변이 안전하고 MCU fault가 없으며 모든 조작 장치가 중립인지 확인하세요. 아래 문구를 입력해야 해제할 수 있습니다.</p>
      <label class="modal-field"><span>확인 문구</span><input id="releasePhrase" autocomplete="off" placeholder="정지 해제" /></label></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmRelease">해제 확인</button></div>`);
    $('#confirmRelease').addEventListener('click', () => {
      if ($('#releasePhrase').value.trim() !== '정지 해제') {
        toast('확인 문구가 일치하지 않습니다', '“정지 해제”를 정확히 입력하세요.', 'warning');
        return;
      }
      state.estop = false;
      addLog('INFO', 'safety_supervisor', 'E-STOP latch released after operator confirmation.');
      closeModal();
      renderGlobal();
      toast('E-STOP이 해제되었습니다', '주행 전 주변 안전을 다시 확인하세요.', 'success');
    });
  }

  function toggleControl() {
    if (state.hasControl) {
      if (state.manual.enabled || state.nav.state === 'RUNNING') {
        toast('제어권을 반납할 수 없습니다', '진행 중인 주행을 먼저 종료하세요.', 'warning');
        return;
      }
      state.hasControl = false;
      toast('제어권을 반납했습니다', '현재 관전 모드입니다.');
    } else {
      state.hasControl = true;
      toast('제어권을 확보했습니다', '조작 기능이 활성화되었습니다.', 'success');
    }
    renderGlobal();
  }

  function renderMappingSteps() {
    const icons = { pending: '○', running: '●', done: '✓', failed: '!', skipped: '−' };
    const labels = { pending: '대기', running: '진행 중', done: '완료', failed: '실패', skipped: '건너뜀' };
    $('#mappingStepper').innerHTML = state.mappingSteps.map((step, index) => `
      <li class="${esc(step.status)}"><span class="step-index">${icons[step.status]}</span><div><strong>${index + 1}. ${esc(step.title)}</strong><small>${esc(step.detail)}</small></div><b>${labels[step.status]}</b></li>`).join('');
  }

  async function startMapping() {
    if (!canOperate()) return;
    if (state.systemState !== 'IDLE') {
      toast('현재 시작할 수 없습니다', `시스템 상태: ${state.systemState}`, 'warning');
      return;
    }
    state.systemState = 'MAPPING';
    state.mappingSaved = false;
    state.mappingElapsed = 0;
    resetMappingSteps();
    $('#startMapping').disabled = true;
    $('#stopMapping').disabled = false;
    $('#mappingStateLabel').textContent = '센서 기동 중';
    state.mappingSteps[0].status = 'running';
    renderMappingSteps(); renderGlobal();
    addLog('INFO', 'alm_web_backend', 'Launching profile sensors.');
    await wait(900);
    if (state.systemState !== 'MAPPING') return;
    state.mappingSteps[0].status = 'done';
    state.mappingSteps[1].status = 'running';
    $('#mappingStateLabel').textContent = '매핑 엔진 기동 중';
    addLog('INFO', 'lidar.launch.py', 'PointCloud and IMU streams confirmed.');
    addLog('INFO', 'alm_web_backend', `Launching ${state.profile === 'C' ? 'mapping_sclio' : 'mapping_fastlio'}.`);
    renderMappingSteps();
    await wait(1100);
    if (state.systemState !== 'MAPPING') return;
    state.mappingSteps[1].status = 'done';
    state.mappingSteps[2].status = 'running';
    $('#mappingStateLabel').textContent = '매핑 진행 중';
    addLog('INFO', 'mapping_engine', 'Mapping active. Vehicle motion source: external joystick.');
    renderMappingSteps();
    state.mappingTimer = setInterval(() => {
      state.mappingElapsed += 1;
      state.pointCount = Math.min(200000, state.pointCount + Math.round(60 + Math.random() * 170));
      $('#mappingElapsed').textContent = fmtDuration(state.mappingElapsed);
      $('#pointCount').textContent = state.pointCount.toLocaleString();
    }, 1000);
    toast('SLAM 매핑을 시작했습니다', '차량 이동은 외부 조이스틱으로 수행하세요.', 'success');
  }

  function stopMapping() {
    if (state.systemState !== 'MAPPING') return;
    const proceed = () => {
      clearInterval(state.mappingTimer);
      state.mappingTimer = null;
      state.systemState = 'IDLE';
      const scan = state.mappingSteps.find((step) => step.key === 'scanning');
      if (scan.status === 'running') scan.status = 'done';
      $('#mappingStateLabel').textContent = state.mappingSaved ? '매핑 완료' : '저장 전 종료';
      $('#startMapping').disabled = false;
      $('#stopMapping').disabled = true;
      addLog('INFO', 'alm_web_backend', 'Mapping launch groups stopped in reverse order.');
      renderMappingSteps(); renderGlobal(); closeModal();
      toast('SLAM 프로세스를 종료했습니다', state.mappingSaved ? '저장된 맵을 사용할 수 있습니다.' : 'PCD가 저장되지 않았습니다.', state.mappingSaved ? 'success' : 'warning');
    };
    if (!state.mappingSaved) {
      openModal(`<div class="modal-head"><div><p class="section-kicker">UNSAVED MAP</p><h2>저장하지 않고 종료할까요?</h2></div><button class="close-button" data-close-modal>×</button></div><p class="modal-copy">현재 누적 맵이 PCD로 저장되지 않았습니다. 종료하면 이번 매핑 결과를 잃을 수 있습니다.</p><div class="modal-actions"><button class="secondary-button" data-close-modal>계속 매핑</button><button class="danger-text-button" id="forceStopMapping">저장 없이 종료</button></div>`);
      $('#forceStopMapping').addEventListener('click', proceed);
    } else proceed();
  }

  async function savePcd() {
    if (!canOperate()) return;
    if (!['MAPPING', 'IDLE'].includes(state.systemState)) return;
    const button = $('#savePcd');
    setButtonBusy(button, true, '저장 중…');
    addLog('INFO', 'map_save', 'Calling /map_save service.');
    await wait(1300);
    state.mappingSaved = true;
    const step = state.mappingSteps.find((item) => item.key === 'save_pcd');
    step.status = 'done';
    const map = state.maps.find((item) => item.name === state.activeMap);
    if (map) map.hasPcd = true;
    $('#pcdAssetText').textContent = `${state.activeMap}.pcd · 18.4 MB · ${state.pointCount.toLocaleString()} pts`;
    addLog('INFO', 'map_save', `Saved /data/maps/${state.activeMap}/${state.activeMap}.pcd`);
    renderMappingSteps(); setButtonBusy(button, false);
    toast('3D 맵을 저장했습니다', `${state.activeMap}.pcd`, 'success');
  }

  function openPcd2Pgm() {
    if (!state.mappingSaved) {
      toast('PCD가 필요합니다', '먼저 3D 맵을 저장하세요.', 'warning');
      return;
    }
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">PCD TO PGM</p><h2>2D 맵 변환</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">높이 밴드를 조정해 장애물 단면을 생성합니다. 작업 진행률은 임의 추정하지 않고 단계 상태로 표시합니다.</p>
      <div class="modal-grid"><label class="modal-field"><span>Resolution</span><input id="pgmResolution" type="number" value="0.05" step="0.01"></label><label class="modal-field"><span>Min points</span><input id="pgmMinPoints" type="number" value="1"></label><label class="modal-field"><span>Z min</span><input id="pgmZMin" type="number" value="-0.3" step="0.1"></label><label class="modal-field"><span>Z max</span><input id="pgmZMax" type="number" value="1.0" step="0.1"></label></div>
      <div class="preview-grid"><div><small>직전 결과</small><div class="preview-map"></div></div><div><small>새 미리보기</small><div class="preview-map" style="filter:contrast(1.15)"></div></div></div><div id="jobStage" class="modal-copy">대기 중</div><div class="modal-progress"><i id="pgmProgress"></i></div></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>닫기</button><button class="primary-button" id="runPgmJob">변환 실행</button></div>`);
    $('#runPgmJob').addEventListener('click', async () => {
      const button = $('#runPgmJob'); setButtonBusy(button, true, '실행 중…');
      const stages = ['PCD 읽는 중', '높이 필터링', '격자화', 'PGM · YAML 저장'];
      for (let i = 0; i < stages.length; i += 1) {
        $('#jobStage').textContent = stages[i];
        $('#pgmProgress').style.width = `${(i + 1) * 25}%`;
        addLog('INFO', 'pcd2pgm', stages[i]);
        await wait(650);
      }
      const step = state.mappingSteps.find((item) => item.key === 'pcd2pgm');
      step.status = 'done';
      const map = state.maps.find((item) => item.name === state.activeMap);
      if (map) map.hasPgm = true;
      $('#pgmAssetText').textContent = `${state.activeMap}.pgm · 412 × 380 · 0.05 m`;
      renderMappingSteps(); setButtonBusy(button, false);
      toast('2D 맵 변환이 완료되었습니다', 'PGM과 YAML을 원자적으로 저장했습니다.', 'success');
    });
  }

  async function buildScDb() {
    if (state.profile === 'A') {
      toast('방식 A에서는 생략됩니다', 'Scan Context DB는 방식 B/C에 필요합니다.');
      return;
    }
    if (!state.mappingSaved) {
      toast('PCD가 필요합니다', '먼저 3D 맵을 저장하세요.', 'warning');
      return;
    }
    const button = $('#buildScDb'); setButtonBusy(button, true, '생성 중…');
    const step = state.mappingSteps.find((item) => item.key === 'sc_db');
    step.status = 'running'; renderMappingSteps();
    addLog('INFO', 'sc_build_db', 'Building descriptors: ring=20 sector=60 radius=10.0m.');
    await wait(2100);
    step.status = 'done';
    const map = state.maps.find((item) => item.name === state.activeMap);
    if (map) map.hasScDb = true;
    $('#scAssetText').textContent = 'sc_db.npz · 312 keyframes · selftest 95%';
    addLog('INFO', 'sc_build_db', 'Self-test success: 19/20.');
    renderMappingSteps(); setButtonBusy(button, false);
    toast('Scan Context DB를 생성했습니다', '312 keyframes · selftest 95%', 'success');
  }

  function openMapManager() {
    const rows = state.maps.map((map) => `
      <div class="map-manager-item"><div><strong>${esc(map.name)} · revision ${esc(map.revision)}</strong><small>PCD hash ${esc(map.hash)}… · ${esc(map.date)}</small><span>${map.hasPcd ? '<b class="cap">PCD</b>' : ''}${map.hasPgm ? '<b class="cap">PGM</b>' : ''}${map.hasScDb ? '<b class="cap">SC DB</b>' : ''}</span></div><button class="small-button" data-use-map="${esc(map.name)}">사용</button></div>`).join('');
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">MAP ASSETS</p><h2>맵 자산 관리</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="map-manager-list">${rows}</div>
      <p class="modal-copy">PGM과 SC DB는 부모 PCD의 hash와 revision이 일치할 때만 사용할 수 있습니다.</p>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>닫기</button><button class="primary-button" id="openNewMapForm">＋ 새 맵 만들기</button></div>`);
    $$('[data-use-map]', $('#modal')).forEach((button) => button.addEventListener('click', () => {
      state.activeMap = button.dataset.useMap;
      const map = state.maps.find((item) => item.name === state.activeMap);
      state.mapRevision = map ? map.revision : 1;
      $('#navMapTitle').textContent = `${state.activeMap} · revision ${state.mapRevision}`;
      closeModal(); renderGlobal();
      toast('활성 맵을 변경했습니다', state.activeMap, 'success');
    }));
    $('#openNewMapForm').addEventListener('click', openNewMapModal);
  }

  function renderMapOptions() {
    $('#settingsMapSelect').innerHTML = state.maps.map((map) => `<option value="${esc(map.name)}">${esc(map.name)}</option>`).join('');
    $('#settingsMapSelect').value = state.activeMap;
  }

  function openNewMapModal() {
    if (state.systemState !== 'IDLE') {
      toast('새 맵을 만들 수 없습니다', '현재 실행 중인 워크플로를 먼저 종료하세요.', 'warning');
      return;
    }
    openModal(`
      <div class="modal-head"><div><p class="section-kicker">NEW MAP</p><h2>새 맵 만들기</h2></div><button class="close-button" data-close-modal>×</button></div>
      <div class="modal-body"><p class="modal-copy">새 이름으로 빈 맵을 만들고 바로 활성화합니다. SLAM을 시작하면 이 이름으로 PCD가 저장됩니다.</p>
      <label class="modal-field"><span>맵 이름</span><input id="newMapName" autocomplete="off" placeholder="예: warehouse_b" /></label>
      <p class="modal-copy text-danger hidden" id="newMapError"></p></div>
      <div class="modal-actions"><button class="secondary-button" data-close-modal>취소</button><button class="primary-button" id="confirmNewMap">만들기</button></div>`);
    $('#newMapName').focus();
    $('#confirmNewMap').addEventListener('click', () => {
      const raw = $('#newMapName').value.trim();
      const errorEl = $('#newMapError');
      if (!/^[a-zA-Z0-9_-]{2,32}$/.test(raw)) {
        errorEl.textContent = '영문·숫자·_·- 조합 2~32자로 입력하세요.';
        errorEl.classList.remove('hidden');
        return;
      }
      if (state.maps.some((map) => map.name.toLowerCase() === raw.toLowerCase())) {
        errorEl.textContent = '이미 존재하는 맵 이름입니다.';
        errorEl.classList.remove('hidden');
        return;
      }
      state.maps.push({ name: raw, revision: 1, hash: Math.random().toString(16).slice(2, 6), date: nowTime(), hasPcd: false, hasPgm: false, hasScDb: false });
      state.activeMap = raw; state.mapRevision = 1; state.mappingSaved = false;
      resetMappingSteps();
      $('#pcdAssetText').textContent = '아직 저장되지 않음';
      $('#pgmAssetText').textContent = 'resolution 0.05 m';
      $('#scAssetText').textContent = 'B/C 자동 측위용';
      $('#navMapTitle').textContent = `${raw} · revision 1`;
      renderMapOptions();
      closeModal(); renderGlobal(); renderLocalization();
      toast('새 맵을 만들었습니다', `${raw} · 활성 맵으로 전환됨`, 'success');
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
      $('#localizationDetail').textContent = `방식 ${state.profile} · ${state.profile === 'A' ? '수동 지정' : 'Scan Context 자동 초기화 가능'}`;
    } else if (loc.state === 'ACCUMULATING') {
      chip.classList.add('running'); chip.textContent = '누적 중';
      $('#localizationTitle').textContent = '라이다 프레임 누적 중';
      $('#localizationDetail').textContent = '로봇을 정지 상태로 유지하세요.';
      $('#localizationProgressText').textContent = `프레임 ${loc.frame} / 10 누적`;
      $('#localizationProgressBar').style.width = `${loc.frame * 10}%`;
    } else if (loc.state === 'MATCHING') {
      chip.classList.add('running'); chip.textContent = '매칭';
      $('#localizationTitle').textContent = 'Scan Context 후보 탐색';
      $('#localizationDetail').textContent = '상위 5개 후보를 계산하고 있습니다.';
      $('#localizationProgressText').textContent = 'Descriptor matching';
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
    $('#localizationModeText').textContent = state.profile === 'A' ? 'Manual' : 'Scan Context';
  }

  async function autoLocalization() {
    if (!canOperate()) return;
    if (state.profile === 'A') {
      toast('방식 A는 수동 초기위치를 사용합니다', '맵에서 위치와 방향을 지정하세요.', 'warning');
      manualInitialPose();
      return;
    }
    if (state.systemState !== 'IDLE') {
      toast('측위를 시작할 수 없습니다', '현재 작업을 먼저 종료하세요.', 'warning');
      return;
    }
    state.systemState = 'LOCALIZING';
    state.localization = { state: 'ACCUMULATING', frame: 0, fitness: null, pose: null };
    $('#candidateLayer').classList.remove('hidden-layer');
    renderLocalization(); renderGlobal();
    addLog('INFO', 'sc_localizer', 'Relocalization requested. Accumulating 10 frames.');
    for (let frame = 1; frame <= 10; frame += 1) {
      await wait(180);
      if (state.localization.state !== 'ACCUMULATING') return;
      state.localization.frame = frame; renderLocalization();
    }
    state.localization.state = 'MATCHING'; renderLocalization(); await wait(900);
    state.localization.state = 'WAITING_ICP';
    $('#candidateLayer').innerHTML = [[280,220],[410,265],[550,350],[620,235],[700,420]].map(([x,y], index) => `<circle cx="${x}" cy="${y}" r="${12-index}" fill="#8c61ff" opacity="${0.85-index*0.1}"/>`).join('');
    renderLocalization(); await wait(1300);
    completeLocalization('scan_context');
  }

  function completeLocalization(mode) {
    state.localization.state = 'CONVERGED';
    state.localization.fitness = mode === 'manual' ? 0.164 : 0.087;
    state.localization.pose ||= { x: -1.6, y: 0.2, yaw: 12 };
    state.systemState = 'IDLE';
    $('#candidateLayer').classList.add('hidden-layer');
    renderLocalization(); renderGlobal();
    addAlarm('info', '초기위치 확정', `ICP fitness ${state.localization.fitness.toFixed(3)}`);
    addLog('INFO', 'sc_localizer', `Localization converged. mode=${mode} fitness=${state.localization.fitness.toFixed(3)}`);
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
    state.profile === 'A' ? manualInitialPose() : autoLocalization();
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

  function commandFor(name) {
    const factor = state.manual.multiplier;
    const mode = state.manual.mode;
    if (name === 'stop') return { x: 0, y: 0, z: 0 };
    if (mode === 'spin') return { x: 0, y: 0, z: name === 'left' ? 0.8 * factor : name === 'right' ? -0.8 * factor : 0 };
    if (mode === 'normal' || mode === 'auto') {
      if (name === 'forward') return { x: 0.45 * factor, y: 0, z: 0 };
      if (name === 'reverse') return { x: -0.15 * factor, y: 0, z: 0 };
      if (name === 'left') return { x: 0, y: 0, z: 0.8 * factor };
      if (name === 'right') return { x: 0, y: 0, z: -0.8 * factor };
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
      profile: state.profile,
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
    const active = $('#profileCards button.active');
    state.profile = active?.dataset.profile || state.profile;
    state.activeMap = $('#settingsMapSelect').value;
    const map = state.maps.find((item) => item.name === state.activeMap);
    state.mapRevision = map ? map.revision : 1;
    resetMappingSteps(); renderGlobal(); renderLocalization();
    $('#navMapTitle').textContent = `${state.activeMap} · revision ${state.mapRevision}`;
    closeSettings();
    toast('설정을 저장했습니다', `방식 ${state.profile} · ${state.activeMap}`, 'success');
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
    $('#buildScDb').addEventListener('click', buildScDb);
    $('#openMapManager').addEventListener('click', openMapManager);
    $('#newMapButton').addEventListener('click', openNewMapModal);
    $('#mappingProfile').addEventListener('change', (event) => {
      if (state.systemState !== 'IDLE') { event.target.value = state.profile; toast('실행 중에는 프로필을 바꿀 수 없습니다', '', 'warning'); return; }
      state.profile = event.target.value; resetMappingSteps(); renderGlobal(); renderLocalization();
    });
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
    $('#profileCards').addEventListener('click', (event) => {
      const button = event.target.closest('button[data-profile]'); if (!button) return;
      $$('#profileCards button').forEach((item) => item.classList.toggle('active', item === button));
    });
    $('#testConnection').addEventListener('click', async (event) => {
      setButtonBusy(event.currentTarget, true, '테스트 중…'); await wait(850); setButtonBusy(event.currentTarget, false);
      toast('연결 테스트 성공', 'Backend 18 ms · Foxglove 24 ms · MCU online', 'success');
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
    addLog, addAlarm, toast,
  };
})();
