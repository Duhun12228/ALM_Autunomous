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

// rcl_interfaces/msg/Log 의 레벨 상수
const ROS_LOG_LEVEL = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'ERROR' };

// ##CONFIRM## 실제 배터리 팩 사양으로 맞출 것. fake_mcu 의 기본값과 동일하게 둔다.
const BATTERY_EMPTY_V = 21.0;
const BATTERY_FULL_V = 25.2;

const $ = (selector) => document.querySelector(selector);

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
  }

  start() {
    const { bridge } = this;
    bridge.subscribe('/alm/jetson_stats', (msg) => this.onJetsonStats(msg));
    bridge.subscribe('/mcu/state', (msg) => this.onMcuState(msg));
    bridge.subscribe('/mcu/command', (msg) => this.onMcuCommand(msg));
    bridge.subscribe('/cmd_arbiter/owner', (msg) => this.onOwner(msg));
    bridge.subscribe('/drive_mode/effective', (msg) => this.onEffectiveMode(msg));
    bridge.subscribe('/rosout', (msg) => this.onRosout(msg));
    bridge.subscribe('/Odometry', (msg) => this.onOdometry(msg));

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
  onMcuCommand(msg) {
    this.commandedEstop = Boolean(msg.emergency_stop);
    this._syncEstop();
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
    if (this.logBudget <= 0) return;
    this.logBudget -= 1;
    this.alm.addLog(
      ROS_LOG_LEVEL[msg.level] ?? 'INFO',
      msg.name ?? 'unknown',
      msg.msg ?? '',
    );
  }
}
