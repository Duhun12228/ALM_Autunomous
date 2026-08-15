/**
 * 라이브 연동 진입점 (Phase 1~2 · 읽기 전용).
 *
 * 로드 순서가 중요하다. index.html 은 app.js 를 일반 스크립트로, 이 파일을
 * <script type="module"> 로 싣는다. 모듈은 defer 라 app.js 본문보다는 뒤에,
 * DOMContentLoaded(=app.js 의 init) 보다는 앞에 실행된다. 그래서 여기서
 * window.ALM_LIVE 를 세워 두면 init() 이 목업 루프를 띄우지 않는다.
 */
import { RosBridge, resolveBridgeUrl } from './bridge/ros-bridge.js';
import { Commands } from './commands.js';
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

  // 쓰기 경로. 읽기(bridge)와 완전히 다른 포트·프로토콜이다.
  const commands = new Commands(alm, {
    onSessionChange: (held) => {
      // state.hasControl 은 서버 판정의 캐시다 — 여기서만 갱신한다.
      alm.state.hasControl = held;
      alm.renderGlobal();
      alm.renderManual();
    },
  });
  window.ALM_CMD = commands;
  // 서버가 제어권을 주기 전까지는 관전 모드다. 목업 기본값(true)을 내린다.
  alm.state.hasControl = false;

  const canvas = document.querySelector('#pointCloudCanvas');
  const renderer3d = canvas ? new Renderer3D(canvas) : null;
  // ingest 가 placeholder 를 띄울 때 렌더 루프를 멈추기 위해 참조를 남긴다
  window.ALM_RENDERER3D = renderer3d;

  ingest.start();

  // ── 시각화 구독 ──────────────────────────────────────────────────
  bridge.subscribe('/map', (msg) => map2d.onOccupancyGrid(msg));
  bridge.subscribe('/Odometry', (msg) => map2d.onOdometry(msg));
  bridge.subscribe('/plan', (msg) => map2d.onPath(msg, '#globalPathLayer', '#6FA8FF', 6));
  bridge.subscribe('/local_plan', (msg) => map2d.onPath(msg, '#localPathLayer', '#4ADE9B', 4));
  bridge.subscribe('/scan', (msg) => map2d.onLaserScan(msg, ingest.latestPose));

  if (renderer3d) {
    // 원본은 '현재 스캔'만, 정합된 것은 '현재 스캔 + 누적 맵'.
    // 누적이 /cloud_registered 전용인 이유는 renderer3d.js 도입부 참조.
    bridge.subscribe('/livox/lidar', (msg) => renderer3d.onLiveCloud(msg));
    bridge.subscribe('/cloud_registered', (msg) => renderer3d.onRegisteredCloud(msg));
    // 활성 맵에 저장된 점군. latched 라 늦게 붙어도 즉시 받는다.
    bridge.subscribe('/alm/prior_cloud', (msg) => renderer3d.onPriorCloud(msg));
    bridge.subscribe('/alm/prior_cloud_map', (msg) => {
      renderer3d.priorMapName = msg.data ?? '';
    });

    // 목업에서 토스트만 띄우던 두 버튼을 실제 카메라 조작으로 바꾼다
    document.querySelector('#reset3d')?.addEventListener(
      'click', () => renderer3d.resetView(), { capture: true });
    document.querySelector('#topView3d')?.addEventListener(
      'click', () => renderer3d.topView(), { capture: true });

    // 툴바의 '누적 맵' / '현재 스캔' 을 실제 레이어에 연결한다.
    // (app.js 는 .active 클래스만 토글하고 있었다 — 화면상 장식이었다)
    document.querySelectorAll('.viewport-toolbar .tool').forEach((button) => {
      button.addEventListener('click', () => setTimeout(() => {
        renderer3d.setLayerVisible(button.dataset.layer,
          button.classList.contains('active'));
      }, 0));
    });

    // 누적 점 수와 '지금 무엇을 보고 있는지' 를 화면에 올린다.
    // (#pointCount 는 목업 숫자를, top-right 는 고정 문구를 그리고 있었다)
    const countNode = document.querySelector('#pointCount');
    const capNode = countNode?.nextElementSibling;
    const sourceNode = document.querySelector('#streamSource');
    const rateNode = document.querySelector('#streamRate');
    const detailNode = document.querySelector('#streamDetail');
    if (capNode) capNode.textContent = `/ ${renderer3d.maxAccumulated.toLocaleString()}`;

    // 매핑 탭이 보일 때만 렌더한다
    const syncViewport = () => {
      const visible = document.querySelector('#tab-mapping')?.classList.contains('active')
        && !document.hidden
        // placeholder 가 떠 있으면(= 그릴 것이 하나도 없으면) rAF 를 돌릴 이유가 없다
        && !ingest.placeholder3dActive;
      renderer3d.setActive(Boolean(visible));
    };
    document.querySelectorAll('.nav-item').forEach((button) => {
      button.addEventListener('click', () => setTimeout(syncViewport, 0));
    });
    document.addEventListener('visibilitychange', syncViewport);
    syncViewport();

    setInterval(() => {
      // placeholder 는 인벤토리(5초)보다 빠르게 바뀌는 조건에 걸려 있다.
      // 매핑을 시작하면 저장된 cloud.pcd 가 없어도 그릴 것이 생기므로,
      // 여기서 다시 판단하고 렌더러를 깨운다.
      ingest.refreshPlaceholders();
      syncViewport();

      const { source, detail } = renderer3d.describeSource();
      if (sourceNode) {
        sourceNode.textContent = renderer3d.priorMapName
          ? `${source} · ${renderer3d.priorMapName}` : source;
      }
      if (rateNode) {
        rateNode.textContent = Number.isFinite(renderer3d.pointsPerSecond)
          ? `${Math.round(renderer3d.pointsPerSecond).toLocaleString()} pts/s` : '—';
      }
      if (detailNode) detailNode.textContent = detail;
      if (!countNode) return;
      countNode.textContent = renderer3d.accumulatedCount.toLocaleString();
      countNode.classList.toggle('text-warning', renderer3d.accumulationSaturated);
      if (renderer3d.accumulationSaturated && !renderer3d.saturationReported) {
        renderer3d.saturationReported = true;
        alm.addAlarm('warning', '누적 맵 버퍼가 찼습니다',
          `${renderer3d.maxAccumulated.toLocaleString()}점(복셀 ${renderer3d.voxelSize} m)에 도달해 `
          + '화면 누적을 멈췄습니다. 로봇 쪽 매핑은 계속 진행됩니다.');
      }
    }, 1000);

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

  // backend 는 명령 경로다. 3초마다 health 로 생존을 확인한다.
  // 토큰이 없으면 '연결 실패'가 아니라 '설정 필요'라고 말해야 한다 — 조작자가
  // 네트워크를 의심하며 시간을 버리게 만들지 않는다.
  const pollBackend = async () => {
    if (!backendDot) return;
    const item = backendDot.closest('.hud-item');
    if (!commands.hasToken()) {
      backendDot.className = 'status-dot warn';
      item?.setAttribute('title', '백엔드 토큰이 없습니다 — 설정에서 입력하세요');
      return;
    }
    try {
      const health = await commands.health();
      backendDot.className = 'status-dot ok';
      item?.setAttribute('title', `alm_web_backend 정상 · 활성 맵 ${health.active_map}`);
      // 매핑 프로세스의 진실은 서버가 갖고 있다. 이 탭에서 시작하지 않았어도
      // (다른 브라우저, CLI, 재접속 전에 시작된 경우) 화면이 알아야 한다.
      // 점군이 실측인지 재생본인지. 토픽 이름으로는 구분이 안 되므로 백엔드가
      // 발행 노드를 조회해 알려준다.
      alm.setLidarSource?.(health.lidar_source);
      const slam = (health.processes || []).find((p) => p.slot === 'slam');
      const running = Boolean(slam?.running);
      if (running !== alm.state.slamRunning) {
        alm.state.slamRunning = running;
        alm.onSlamRunningChange?.(running, slam);
      }
      // 측위도 같은 이유로 감시한다 — CLI 로 localization.launch.py 를 먼저
      // 띄워두고 웹을 여는 것은 흔한 순서다.
      //
      // 슬롯과 그래프를 **둘 다** 본다. 슬롯만 보면 웹 밖에서 띄운 스택을
      // '안 돌고 있음'으로 표시하고, 그래프만 보면 웹이 내릴 수 있는 것인지를
      // 알 수 없다. external = 돌고 있는데 웹이 띄운 게 아니다 (= 못 내린다).
      const loc = (health.processes || []).find((p) => p.slot === 'localization');
      const nodes = health.localization_nodes || [];
      const locRunning = Boolean(loc?.running) || nodes.length > 0;
      const external = !loc?.running && nodes.length > 0;
      if (locRunning !== alm.state.localization.running
          || external !== alm.state.localization.external) {
        alm.onLocalizationRunningChange?.(locRunning, loc, { external, nodes });
      }
    } catch (error) {
      backendDot.className = 'status-dot danger';
      item?.setAttribute('title', `alm_web_backend: ${error.message}`);
    }
  };
  pollBackend();
  setInterval(pollBackend, 3000);

  // command_manager 의 실제 속도 한계로 UI 하드코딩을 덮는다 (§12-7)
  commands.limits()
    .then((result) => alm.applyLimits(result.limits))
    .catch(() => { /* 백엔드가 아직 없으면 기본값을 그대로 쓴다 */ });

  // 탭을 닫을 때 제어권을 반납한다 (실패해도 서버 리스가 정리한다)
  window.addEventListener('pagehide', () => commands.releaseControlOnUnload());

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
 *
 * 매핑 5개와 E-STOP 은 이제 실제로 로봇에 나간다(Phase 3). 측위(자동 탐색·
 * 재측위)도 Phase 4-1 에서 붙었다. 여기 남은 것들은 여전히 화면 안에서만 도는
 * 것들이다 — 실데이터 패널과 섞여 있으면 조작자가 "눌렀으니 로봇이 반응했겠지"로
 * 오해하므로 배지를 유지한다.
 *
 * ⚠ #manualInitialPose 는 '아직 안 붙였다'가 아니라 **붙일 대상이 없다.**
 *   이 브랜치의 teaser_fpfh_localizer 는 /initialpose 를 구독하지 않는다
 *   (구독하는 것은 쓰지 않는 icp_node 다). 전역 정합이라 초기 추정 자체가
 *   필요 없는 방식이므로, 배지를 유지하고 누르면 그렇게 말한다.
 *
 * ⚠ 수동주행(enterManual/exitManual)은 단순히 '아직 안 붙였다'가 아니다.
 *   base_control.yaml 의 4WIS 변환 상수 8개가 ##CONFIRM## 미확정이라, 지금
 *   twist 를 보내면 그대로 엉뚱한 rpm/조향각이 된다. 실차 확정 전에는 붙이면
 *   안 되는 순서다 (JETSON_INTEGRATION_PLAN.md §9 Phase 5).
 */
function markMockControls() {
  const mockSelectors = [
    ['#manualInitialPose', 'manual-pose'],
    ['#startNavigation', 4], ['#pauseNavigation', 4], ['#cancelNavigation', 4],
    ['#enterManual', 5], ['#exitManual', 5],
  ];
  const TITLES = {
    'manual-pose': '연결 대상이 없습니다 — FPFH+TEASER++ 는 초기 추정 없이 전역에서 찾습니다',
    4: '아직 목업입니다 — 자율주행 연동은 다음 단계입니다',
    5: '아직 목업입니다 — 수동주행은 4WIS 변환 상수를 실차로 확정한 뒤에 붙입니다',
  };
  for (const [selector, phase] of mockSelectors) {
    const node = document.querySelector(selector);
    if (!node) continue;
    node.classList.add('is-mock');
    node.title = TITLES[phase];
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', start);
} else {
  start();
}
