/**
 * 라이브 연동 진입점 (Phase 1~2 · 읽기 전용).
 *
 * 로드 순서가 중요하다. index.html 은 app.js 를 일반 스크립트로, 이 파일을
 * <script type="module"> 로 싣는다. 모듈은 defer 라 app.js 본문보다는 뒤에,
 * DOMContentLoaded(=app.js 의 init) 보다는 앞에 실행된다. 그래서 여기서
 * window.ALM_LIVE 를 세워 두면 init() 이 목업 루프를 띄우지 않는다.
 */
import { RosBridge, resolveBridgeUrl } from './bridge/ros-bridge.js';
import { Ingest } from './ingest.js';
import { Map2D } from './render/map2d.js';
import { Renderer3D } from './render/renderer3d.js';

// app.js 의 init() 보다 먼저 세워야 한다 (파일 최상단에서 실행되는 이유)
window.ALM_LIVE = true;

function start() {
  const alm = window.ALM;
  if (!alm) {
    console.error('[alm] app.js 가 window.ALM 을 노출하지 않았습니다. 스크립트 순서를 확인하세요.');
    return;
  }

  const url = resolveBridgeUrl();
  const bridge = new RosBridge(url);
  const ingest = new Ingest(bridge, alm);
  const map2d = new Map2D();

  const canvas = document.querySelector('#pointCloudCanvas');
  const renderer3d = canvas ? new Renderer3D(canvas) : null;

  ingest.start();

  // ── 시각화 구독 ──────────────────────────────────────────────────
  bridge.subscribe('/map', (msg) => map2d.onOccupancyGrid(msg));
  bridge.subscribe('/Odometry', (msg) => map2d.onOdometry(msg));
  bridge.subscribe('/plan', (msg) => map2d.onPath(msg, '#globalPathLayer', '#6FA8FF', 6));
  bridge.subscribe('/local_plan', (msg) => map2d.onPath(msg, '#localPathLayer', '#4ADE9B', 4));
  bridge.subscribe('/scan', (msg) => map2d.onLaserScan(msg, ingest.latestPose));

  if (renderer3d) {
    bridge.subscribe('/livox/lidar', (msg) => renderer3d.onPointCloud(msg));
    bridge.subscribe('/cloud_registered', (msg) => renderer3d.onPointCloud(msg));

    // 목업에서 토스트만 띄우던 두 버튼을 실제 카메라 조작으로 바꾼다
    document.querySelector('#reset3d')?.addEventListener(
      'click', () => renderer3d.resetView(), { capture: true });
    document.querySelector('#topView3d')?.addEventListener(
      'click', () => renderer3d.topView(), { capture: true });

    // 매핑 탭이 보일 때만 렌더한다
    const syncViewport = () => {
      const visible = document.querySelector('#tab-mapping')?.classList.contains('active')
        && !document.hidden;
      renderer3d.setActive(Boolean(visible));
    };
    document.querySelectorAll('.nav-item').forEach((button) => {
      button.addEventListener('click', () => setTimeout(syncViewport, 0));
    });
    document.addEventListener('visibilitychange', syncViewport);
    syncViewport();

    setInterval(() => ingest.setPointsRate(renderer3d.pointsPerSecond), 1000);
  }

  // ── HUD 연결 표시 ────────────────────────────────────────────────
  const bridgeDot = document.querySelector('.hud-group .hud-item:nth-child(1) .status-dot');
  const backendDot = document.querySelector('.hud-group .hud-item:nth-child(2) .status-dot');
  const mcuDot = document.querySelector('.hud-group .hud-item:nth-child(3) .status-dot');

  bridge.onStatus((status) => {
    if (bridgeDot) bridgeDot.className = `status-dot ${status === 'connected' ? 'ok' : 'danger'}`;
    if (status === 'connected') {
      alm.addLog('INFO', 'foxglove_bridge', `연결됨 — ${url}`);
    } else if (status === 'disconnected') {
      alm.addLog('WARN', 'foxglove_bridge', '연결 끊김 — 재접속 시도 중');
    }
  });

  // backend 는 Phase 3 에서 생긴다. 지금은 없는 게 정상이므로 회색으로 둔다.
  if (backendDot) {
    backendDot.className = 'status-dot';
    backendDot.closest('.hud-item')?.setAttribute(
      'title', 'alm_web_backend 는 Phase 3(명령 연동)에서 추가됩니다');
  }

  // /mcu/state 가 실제로 갱신되는지로 MCU 생존을 판정한다
  setInterval(() => {
    if (mcuDot) {
      mcuDot.className = `status-dot ${bridge.isFresh('/mcu/state', 3000) ? 'ok' : 'danger'}`;
    }
  }, 1000);

  markMockControls();
  bridge.connect();
  window.ALM_BRIDGE = bridge;   // 콘솔 디버깅용
}

/**
 * 아직 mock 인 조작 계열에 배지를 붙인다.
 * 읽기 전용 단계에서 이 버튼들은 화면 안에서만 동작한다 — 실데이터 패널과
 * 섞여 있으면 조작자가 "눌렀으니 로봇이 반응했겠지"로 오해한다.
 */
function markMockControls() {
  const mockSelectors = [
    '#startMapping', '#stopMapping', '#savePcd', '#openPcd2Pgm', '#buildScDb',
    '#autoLocalization', '#manualInitialPose', '#relocalize',
    '#startNavigation', '#pauseNavigation', '#cancelNavigation',
    '#enterManual', '#exitManual',
  ];
  for (const selector of mockSelectors) {
    const node = document.querySelector(selector);
    if (!node) continue;
    node.classList.add('is-mock');
    node.title = '아직 목업입니다 — 명령 연동은 Phase 3부터';
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', start);
} else {
  start();
}
