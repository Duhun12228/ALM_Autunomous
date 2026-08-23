/**
 * 구독한 ROS 메시지를 화면에 반영한다 (Phase 1 · 읽기 전용).
 *
 * 원칙: app.js 의 render*() 를 최대한 재사용하고, 그 함수가 **목업이라서 지어낸
 * 값**만 뒤에서 실측으로 덮는다. 예를 들어 renderMetrics() 는 링·막대를
 * state.metrics 로 정확히 그리지만, RAM 총량을 7.8 GB 로 박아두고 온도 4종을
 * 하나의 temp 에서 산술로 만들어낸다 — 그 부분만 다시 쓴다.
 *
 * 여기서 나가는 것은 아무것도 없다. 로봇으로 가는 경로는 Phase 3 의
 * alm_web_backend 가 생기기 전까지 존재하지 않는다.
 */
import { quaternionToYaw } from './bridge/decoders.js';
import { toggleMapPlaceholder, toggleSvgMapPlaceholder } from './render/placeholder.js';

// rcl_interfaces/msg/Log 의 레벨 상수
const ROS_LOG_LEVEL = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'ERROR' };

/**
 * 측위 스택을 이루는 노드들. 이 이름들의 로그는 전역 예산과 **별도로** 통과시킨다.
 *
 * 이유: 전역 logBudget 은 초당 20건을 모든 노드가 나눠 쓴다. 정합 중에는
 * fast_lio 와 livox 파서가 훨씬 시끄러워서, 정작 봐야 할 teaser 줄이 예산에
 * 밀려 조용히 버려질 수 있다. 정합 로그는 시도당 몇 줄이라 따로 빼도 부담이 없다.
 */
const LOCALIZATION_NODES = new Set([
  'teaser_fpfh_localizer', 'transform_publisher', 'fastlio_localization',
]);

/** teaser_fpfh_localizer 가 쓰는 [단계] 태그. 소스의 로그 문자열과 1:1 이다. */
const LOC_STAGE_TAGS = ['ACCUM', 'ATTEMPT', 'FPFH', 'MATCH', 'TEASER', 'GICP', 'CONSISTENCY'];

// ##CONFIRM## 실제 배터리 팩 사양으로 맞출 것. fake_mcu 의 기본값과 동일하게 둔다.
const BATTERY_EMPTY_V = 21.0;
const BATTERY_FULL_V = 25.2;

const $ = (selector) => document.querySelector(selector);

/** 자산 종류 → 화면의 자산 카드. map_manager 의 kind 와 1:1 이다. */
const ASSET_CARD = {
  cloud: { text: '#pcdAssetText', label: '3D 맵' },
  grid: { text: '#pgmAssetText', label: '2D 맵' },
  fpfh: { text: '#fpfhAssetText', label: '측위 DB' },
};

const bytesToText = (bytes) => {
  if (!Number.isFinite(bytes) || bytes <= 0) return '';
  const mb = bytes / (1024 * 1024);
  return mb >= 1 ? `${mb.toFixed(1)} MB` : `${(bytes / 1024).toFixed(0)} KB`;
};

const setText = (selector, text) => {
  const node = $(selector);
  if (node) node.textContent = text;
};

const setWidth = (selector, percent) => {
  const node = $(selector);
  if (node) node.style.width = `${Math.max(0, Math.min(100, percent))}%`;
};

/** NaN/undefined 를 화면에서 '—' 로 보이게 한다. 지어낸 0 을 그리지 않기 위함. */
const num = (value, digits = 1, suffix = '') =>
  (typeof value === 'number' && Number.isFinite(value))
    ? `${value.toFixed(digits)}${suffix}`
    : '—';

export class Ingest {
  constructor(bridge, alm) {
    this.bridge = bridge;
    this.alm = alm;
    this.lastFault = { active: false, code: 0 };
    this.lastEstop = false;
    this.lastOwner = null;
    this.logBudget = 0;
    this.lastStaleKey = null;
  }

  start() {
    const { bridge } = this;
    bridge.subscribe('/alm/jetson_stats', (msg) => this.onJetsonStats(msg));
    bridge.subscribe('/alm/map_inventory', (msg) => this.onMapInventory(msg));
    bridge.subscribe('/mcu/state', (msg) => this.onMcuState(msg));
    bridge.subscribe('/mcu/command', (msg) => this.onMcuCommand(msg));
    bridge.subscribe('/cmd_arbiter/owner', (msg) => this.onOwner(msg));
    bridge.subscribe('/drive_mode/effective', (msg) => this.onEffectiveMode(msg));
    bridge.subscribe('/rosout', (msg) => this.onRosout(msg));
    bridge.subscribe('/Odometry', (msg) => this.onOdometry(msg));
    // 측위 수렴. teaser_fpfh_localizer 가 성공했을 때 딱 한 번 낸다.
    bridge.subscribe('/icp_result', (msg) => this.onIcpResult(msg));

    // /rosout 은 노드가 많으면 초당 수십 건도 들어온다. 렌더를 매 건 돌리면
    // 로그창이 화면을 잡아먹으므로 초당 처리량에 상한을 둔다.
    setInterval(() => { this.logBudget = 20; }, 1000);
  }

  // ── 리소스 (실측) ──────────────────────────────────────────────────
  onJetsonStats(msg) {
    const { state } = this.alm;
    const ramPercent = msg.ram_total_gb > 0
      ? (msg.ram_used_gb / msg.ram_total_gb) * 100 : NaN;

    // 링·막대는 app.js 가 state.metrics 로 정확히 그린다
    state.metrics.cpu = msg.cpu_percent;
    state.metrics.gpu = msg.gpu_percent;
    state.metrics.ram = ramPercent;
    state.metrics.temp = msg.temp_cpu;
    state.metrics.power = msg.power_w;
    state.chart.push({ cpu: msg.cpu_percent, gpu: msg.gpu_percent, ram: ramPercent });
    if (state.chart.length > 80) state.chart.shift();

    this.alm.renderMetrics();
    this.alm.drawSystemChart();

    // ── 여기부터는 renderMetrics() 가 지어낸 값을 실측으로 덮는다 ──
    // RAM 총량이 7.8 GB 로 박혀 있다 (실측 7.43 GB)
    setText('#monRam', `${num(msg.ram_used_gb)} / ${num(msg.ram_total_gb)} GB`);
    // 온도 4종을 temp 하나에서 ±상수로 만들어낸다 — 전부 실제 존재하는 센서다
    setText('#tempCpu', num(msg.temp_cpu, 1, '°C'));
    setText('#tempGpu', num(msg.temp_gpu, 1, '°C'));
    setText('#tempSoc', num(msg.temp_soc, 1, '°C'));
    setText('#tempTj', num(msg.temp_tj, 1, '°C'));
    setText('#monPower', num(msg.power_w, 1, ' W'));
    // 네트워크는 통째로 난수였다
    setText('#netTx', num(msg.net_tx_mbps, 1, ' Mbps'));
    setText('#netRx', num(msg.net_rx_mbps, 1, ' Mbps'));
  }

  // ── 맵 자산 (maps/ 실제 상태) ───────────────────────────────────────
  /**
   * map_manager 가 파일시스템에서 본 것을 그대로 화면에 옮긴다.
   * 여기서 값을 지어내지 않는 것이 요점이다 — 목업 시절 이 화면은 맵이 이미
   * 다 있는데도 "아직 저장되지 않음"이라고 말하고 있었다.
   */
  onMapInventory(msg) {
    const { state } = this.alm;
    const maps = msg.maps ?? [];
    state.mapInventory = { root: msg.root, fingerprintVerified: msg.fingerprint_verified };
    state.maps = maps;
    state.activeMap = msg.active_map || '—';

    const active = maps.find((map) => map.active) ?? null;
    this.activeMapEntry = active;

    this._renderAssetCards(active);
    this._renderCapabilities(active);
    this._renderReadyBanner(active);
    this._renderStepper(active);
    this._syncPlaceholders(active);
    this._announceStale(active);

    this.alm.renderGlobal();
    this.alm.renderMapOptions?.();
    setText('#navMapTitle', active
      ? `${active.label || active.name}${active.complete ? '' : ' · 미완성'}`
      : '맵 없음');
  }

  asset(entry, kind) {
    return entry?.assets?.find((item) => item.kind === kind) ?? null;
  }

  _renderAssetCards(entry) {
    for (const [kind, card] of Object.entries(ASSET_CARD)) {
      const node = $(card.text);
      if (!node) continue;
      const row = node.closest('.asset-row');
      const asset = this.asset(entry, kind);

      if (!entry) {
        node.textContent = '맵 목록을 읽는 중…';
      } else if (!asset || !asset.present) {
        node.textContent = asset?.issue || '아직 만들어지지 않음';
      } else if (asset.stale) {
        node.textContent = asset.issue || '부모 PCD와 짝이 맞지 않음';
      } else {
        const size = bytesToText(Number(asset.size_bytes));
        node.textContent = [asset.detail, size].filter(Boolean).join(' · ');
      }
      if (row) {
        row.classList.toggle('is-missing', Boolean(entry) && !asset?.present);
        row.classList.toggle('is-stale', Boolean(asset?.present && asset.stale));
      }
    }
  }

  /** 이 맵으로 실제 무엇을 할 수 있는가 — 자산이 있어야 되는 일만 켠다. */
  _renderCapabilities(entry) {
    const node = $('#mapCapabilities');
    if (!node) return;
    const usable = (kind) => {
      const asset = this.asset(entry, kind);
      return Boolean(asset?.present && !asset.stale);
    };
    const items = [
      ['자동 측위', usable('cloud') && usable('fpfh')],
      ['Nav2 주행', usable('grid')],
      ['3D 뷰', usable('cloud')],
    ];
    node.innerHTML = items.map(([label, ok]) =>
      `<span class="${ok ? 'ok' : ''}">${this.alm.esc(label)}</span>`).join('');
  }

  /** 이미 완성된 맵이라는 사실과, 재매핑이 파괴적이라는 사실을 같이 알린다. */
  _renderReadyBanner(entry) {
    const anchor = $('#mappingStepper');
    if (!anchor) return;
    let banner = document.querySelector('#mapReadyBanner');
    if (!entry) {
      if (banner) banner.remove();
      return;
    }
    if (!banner) {
      banner = document.createElement('p');
      banner.id = 'mapReadyBanner';
      anchor.parentNode.insertBefore(banner, anchor);
    }
    const stale = (entry.assets ?? []).filter((a) => a.present && a.stale);
    if (entry.complete) {
      banner.className = 'map-ready-banner';
      banner.innerHTML = '<b>이 맵은 이미 완성되어 있습니다.</b> '
        + '다시 매핑하면 기존 cloud.pcd 를 덮어씁니다.';
    } else if (stale.length) {
      banner.className = 'map-ready-banner warn';
      banner.textContent = stale[0].issue;
    } else {
      banner.className = 'map-ready-banner warn';
      const missing = (entry.assets ?? []).filter((a) => !a.present)
        .map((a) => ASSET_CARD[a.kind]?.label ?? a.kind);
      banner.innerHTML = `<b>아직 완성되지 않은 맵입니다.</b> 없는 자산: ${missing.join(' · ')}`;
    }
  }

  /**
   * 이미 만들어진 자산에 해당하는 워크플로 단계는 done 으로 시작한다.
   * (mappingSteps 는 app.js 가 소유하므로 status 만 손대고 렌더는 맡긴다.)
   */
  _renderStepper(entry) {
    const steps = this.alm.state.mappingSteps ?? [];
    if (!steps.length) return;
    const done = {
      save_pcd: Boolean(this.asset(entry, 'cloud')?.present),
      pcd2pgm: Boolean(this.asset(entry, 'grid')?.present),
      fpfh_db: Boolean(this.asset(entry, 'fpfh')?.present),
    };
    let touched = false;
    for (const step of steps) {
      if (!(step.key in done)) continue;
      const next = done[step.key] ? 'done' : 'pending';
      // 진행 중인 작업의 상태는 건드리지 않는다
      if (step.status === 'running') continue;
      if (step.status !== next) { step.status = next; touched = true; }
    }
    if (touched) this.alm.renderMappingSteps?.();
  }

  /** 인벤토리에서 온 사실만 기록한다. 실제 표시 판단은 refreshPlaceholders(). */
  _syncPlaceholders(entry) {
    this._placeholder = {
      hasEntry: Boolean(entry),
      hasCloud: Boolean(this.asset(entry, 'cloud')?.present),
      hasGrid: Boolean(this.asset(entry, 'grid')?.present),
      name: entry?.name ?? '',
    };
    this.refreshPlaceholders();
  }

  /**
   * placeholder 표시 여부를 다시 판단한다. 매핑 상태가 인벤토리(5초)보다 빠르게
   * 바뀌므로 main.js 가 1초마다 다시 부른다.
   *
   * 규칙은 두 줄이다.
   *
   *   활성 맵에 cloud.pcd 가 없으면  →  placeholder
   *   단, SLAM 이 도는 중이면        →  누적 점군을 보여준다 (placeholder 없음)
   *
   * 예외가 SLAM 하나뿐인 이유: 그때만 화면에 '만들어지고 있는 맵'이 있다.
   * 라이다가 켜져 있다는 것만으로는 예외가 안 된다 — 실운용에서 라이다는 늘
   * 켜져 있으므로, 그걸 조건에 넣으면 placeholder 가 영영 안 뜬다.
   * (실제로 그렇게 만들었다가 빈 맵으로 바꿔도 안 뜨는 상태가 됐다)
   *
   * 저장하지 않고 SLAM 을 끝내면 누적은 버려지고 다시 placeholder 가 뜬다.
   * 저장했다면 cloud.pcd 가 생겼으므로 애초에 조건에 안 걸린다.
   */
  refreshPlaceholders() {
    const info = this._placeholder;
    if (!info) return;
    const state = this.alm.state;
    const mapping = state.slamRunning === true || state.systemState === 'MAPPING';
    const show3d = info.hasEntry && !info.hasCloud && !mapping;

    const stage = document.querySelector('.pointcloud-stage');
    const showing3d = toggleMapPlaceholder(stage, show3d, 'cloud', info.name);
    // placeholder 가 떠 있으면 three.js 는 그릴 것이 없다 — rAF 를 돌릴 이유도 없다
    if (showing3d) window.ALM_RENDERER3D?.setActive(false);
    this.placeholder3dActive = showing3d;

    // 2D 는 사정이 다르다. pcd2pgm 을 돌리기 전까지 실시간으로 채워지는 것이
    // 없으므로, grid 가 없으면 그대로 placeholder 가 맞다.
    toggleSvgMapPlaceholder(
      document.querySelector('#navigationMap'),
      info.hasEntry && !info.hasGrid, info.name);
  }

  /** stale 은 상승 에지에서만 경보한다 (MCU fault 와 같은 규칙). */
  _announceStale(entry) {
    const stale = (entry?.assets ?? []).filter((a) => a.present && a.stale);
    const key = stale.map((a) => `${a.kind}:${a.issue}`).join('|');
    if (key === this.lastStaleKey) return;
    this.lastStaleKey = key;
    for (const asset of stale) {
      this.alm.addAlarm('warning',
        `${ASSET_CARD[asset.kind]?.label ?? asset.kind} 불일치`, asset.issue);
    }
  }

  /** 포인트 처리량은 renderer3d 가 실제로 받은 프레임에서 계산해 넘겨준다. */
  setPointsRate(pointsPerSecond) {
    setText('#pointsRate', Number.isFinite(pointsPerSecond)
      ? `${Math.round(pointsPerSecond).toLocaleString()} pts/s` : '—');
  }

  // ── MCU 상태 ───────────────────────────────────────────────────────
  onMcuState(msg) {
    const { state } = this.alm;

    // 배터리: 전압에서 역산. renderMetrics() 는 반대로 %에서 전압을 지어낸다.
    const ratio = (msg.battery_voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V);
    if (Number.isFinite(ratio)) {
      state.metrics.battery = Math.max(0, Math.min(100, ratio * 100));
    }
    setText('#batteryVoltage', num(msg.battery_voltage, 1, ' V'));
    setText('#batteryCurrent', num(msg.battery_current, 1, ' A'));

    // 수동주행 탭 텔레메트리 (읽기). 목업 명령 경로가 이 값을 덮어쓸 수 있지만
    // 10 Hz 로 다시 채워지므로 탭 전환 순간의 깜빡임 정도로 끝난다.
    const vx = msg.measured_velocity?.linear?.x ?? NaN;
    const wz = msg.measured_velocity?.angular?.z ?? NaN;
    setText('#measuredX', num(vx, 2));
    setText('#measuredZ', num(wz, 2));
    setWidth('#measuredXBar', Math.abs(vx) / 0.45 * 100);
    setWidth('#measuredZBar', Math.abs(wz) / 0.8 * 100);

    const wheels = msg.wheel_speed ?? [];
    ['#speedFL', '#speedFR', '#speedRL', '#speedRR'].forEach((selector, index) => {
      setText(selector, num(wheels[index], 2));
    });

    // 조향은 축당 1개다 — [front, rear]. UI 의 바퀴 4개는 축 값을 공유한다.
    const steer = msg.steer_angle ?? [];
    const frontDeg = (steer[0] ?? NaN) * 180 / Math.PI;
    const rearDeg = (steer[1] ?? NaN) * 180 / Math.PI;
    setText('#frontSteer', num(frontDeg, 0, '°'));
    setText('#rearSteer', num(rearDeg, 0, '°'));
    this._rotateWheel('#wheelFL', frontDeg);
    this._rotateWheel('#wheelFR', frontDeg);
    this._rotateWheel('#wheelRL', rearDeg);
    this._rotateWheel('#wheelRR', rearDeg);

    setText('#motorState', msg.motors_enabled ? '활성' : '비활성');
    const motorDot = $('#motorDot');
    if (motorDot) motorDot.className = `status-dot ${msg.motors_enabled ? 'ok' : ''}`;

    this.mcuOwnEstop = Boolean(msg.emergency_stop);
    this._syncEstop();

    // renderGlobal() 의 고정 문구를 실제 사유로 교체
    const reasons = [];
    if (this.mcuOwnEstop) reasons.push('MCU E-STOP');
    if (this.commandedEstop) reasons.push('명령 정지');
    if (msg.command_timeout) reasons.push('명령 타임아웃');
    if (msg.fault) reasons.push(`MCU fault ${msg.fault_code}`);
    setText('#safetySub', reasons.length
      ? reasons.join(' · ')
      : 'MCU 정상 · fault 없음 · 명령 수신 중');

    // fault 는 상승 에지에서만 경보를 남긴다 (10 Hz 로 쌓이면 목록이 잠긴다)
    const fault = Boolean(msg.fault);
    if (fault && (!this.lastFault.active || this.lastFault.code !== msg.fault_code)) {
      this.alm.addAlarm('critical', `MCU fault ${msg.fault_code}`,
        msg.fault_text || '상세 정보 없음');
    } else if (!fault && this.lastFault.active) {
      this.alm.addAlarm('info', 'MCU fault 해제', '정상 상태로 복귀했습니다.');
    }
    this.lastFault = { active: fault, code: msg.fault_code };
  }

  /**
   * /mcu/command 의 estop — 조작자나 상위 로직이 '건' 정지.
   * /mcu/state 의 estop 은 MCU 자신의 상태(물리 버튼)라 서로 다르다.
   * 둘 중 하나라도 서 있으면 차량은 멈춘 것이므로 화면에서는 합쳐서 보여준다.
   */
  /**
   * /mcu/command — 로봇에 **실제로 나간** 명령.
   *
   * 두 가지를 본다.
   *   emergency_stop  명령 정지 여부 (MCU 자체 정지와 구분해 표시한다)
   *   speed_rpm       수동주행 탭의 '실제' 칸. 요청과 갈릴 수 있다 —
   *                   모드 전환 dwell 동안 command_manager 가 구동을 0 으로
   *                   잡기 때문이다. 그 차이를 안 보여주면 조작자는
   *                   '밀었는데 안 간다' 로만 겪는다.
   */
  onMcuCommand(msg) {
    this.commandedEstop = Boolean(msg.emergency_stop);
    this._syncEstop();
    this.alm.state.manual.actualRpm = Number(msg.speed_rpm) || 0;
    if (this.alm.state.tab === 'manual') this.alm.updateManualTelemetry?.();
  }

  _syncEstop() {
    const { state } = this.alm;
    const estop = Boolean(this.mcuOwnEstop || this.commandedEstop);
    if (estop !== this.lastEstop) {
      state.estop = estop;
      this.alm.renderGlobal();
      const source = this.mcuOwnEstop ? 'MCU 자체 정지(물리)' : '명령 정지';
      this.alm.addAlarm(estop ? 'critical' : 'info',
        estop ? 'E-STOP 활성화' : 'E-STOP 해제',
        estop ? source : '정지 조건이 모두 해제되었습니다.');
      this.lastEstop = estop;
    }
    setText('#monEstop', estop ? '활성' : '해제');
  }

  _rotateWheel(selector, degrees) {
    const node = $(selector);
    if (node) node.style.transform = Number.isFinite(degrees) ? `rotate(${degrees}deg)` : '';
  }

  // ── 동작권 (cmd_arbiter) ───────────────────────────────────────────
  onOwner(msg) {
    const owner = msg.data ?? '';
    if (owner === this.lastOwner) return;
    this.lastOwner = owner;

    const badge = $('#ownerBadge');
    if (badge) {
      const held = owner.includes('held');
      badge.dataset.owner = held ? 'held' : owner;
      badge.querySelector('strong').textContent =
        held ? '정지 유지' : owner === 'teleop' ? '수동' : '자율';
      badge.title = held
        ? 'teleop(held) — 텔레옵 명령이 끊겨 0 twist 를 유지합니다. '
          + 'set_owner("auto") 를 명시 호출해야 자율로 돌아갑니다.'
        : `cmd_arbiter owner = ${owner}`;
    }
    this.alm.addLog('INFO', 'cmd_arbiter', `동작권 → ${owner}`);
  }

  onEffectiveMode(msg) {
    setText('#effectiveMode', msg.data ?? '—');
    // 자율주행 패널의 '모드' 칸도 이 값을 쓴다. DOM 에서 다시 읽지 않고
    // state 에 둔다 — 화면 요소를 상태 저장소로 쓰면 렌더 순서에 걸린다.
    this.alm.state.driveMode = { effective: msg.data ?? '' };
  }

  // ── 위치 ───────────────────────────────────────────────────────────
  onOdometry(msg) {
    const pose = msg.pose?.pose;
    if (!pose) return;
    this.latestPose = {
      x: pose.position.x,
      y: pose.position.y,
      yaw: quaternionToYaw(pose.orientation),
    };
  }

  // ── 로그 ───────────────────────────────────────────────────────────
  onRosout(msg) {
    const level = ROS_LOG_LEVEL[msg.level] ?? 'INFO';
    const node = msg.name ?? 'unknown';
    const text = msg.msg ?? '';

    // 측위 로그는 전역 예산을 **거치지 않는다** (LOCALIZATION_NODES 주석 참조).
    // 전역 로그창에도 그대로 넣는다 — 매핑 탭에서 보던 흐름이 끊기면 안 된다.
    if (LOCALIZATION_NODES.has(node)) {
      this.alm.addLocalizationLog?.({ level, node, text, stage: locStage(text) });
    }

    if (this.logBudget <= 0) return;
    this.logBudget -= 1;
    this.alm.addLog(level, node, text);
  }

  /**
   * /icp_result — 정합 성공. 이것이 도착했다는 사실 자체가 성공 신호다.
   *
   * 목업은 타이머로 1.3초 뒤에 '수렴'을 선언하고 fitness 0.087 을 지어냈다.
   * 이제는 로봇이 실제로 pose 를 낼 때까지 아무 말도 하지 않는다.
   */
  onIcpResult(msg) {
    const pose = msg?.pose?.pose;
    if (!pose) return;
    this.alm.onLocalizationConverged?.({
      x: pose.position.x,
      y: pose.position.y,
      z: pose.position.z,
      yaw: (quaternionToYaw(pose.orientation) * 180) / Math.PI,
      frame: msg.header?.frame_id ?? 'map',
    });
  }
}

/** 로그 본문 앞머리의 [단계] 태그. 없으면 빈 문자열. */
function locStage(text) {
  const match = /^\[([A-Z]+)(?:\s+\d+)?\]/.exec(text || '');
  if (!match) return '';
  return LOC_STAGE_TAGS.includes(match[1]) ? match[1] : '';
}
