/**
 * 3D 뷰포트 — PointCloud2 를 WebGL 로 그린다.
 *
 * 목업의 drawPointCloud() 는 Canvas 2D 로 sin/cos 점을 매 프레임 1700개씩
 * 찍었다(탭과 무관하게 상시 실행). 여기서는 실제 점군을 GPU 버퍼에 올리고,
 * 탭이 보이지 않으면 렌더 루프 자체를 멈춘다 — Jetson CPU 예산 때문이다.
 *
 * PointCloud2 는 필드 오프셋이 메시지마다 다를 수 있으므로 헤더에서 읽어
 * DataView 로 직접 훑는다. livox_udp_pointcloud2 는 x,y,z,intensity,time
 * (전부 float32, point_step 20) 을 쓴다.
 *
 * ── 레이어 셋 ─────────────────────────────────────────────────────────
 *
 *   prior       활성 맵에 **저장돼 있는** 점군. prior_cloud_publisher 가 복셀
 *               다운샘플해 latched 로 보낸 것. 맵을 바꾸면 이것만 바뀐다.
 *   live        마지막 스캔 한 장. 매번 덮어쓴다.
 *   accumulated 매핑 중 쌓이는 맵. 복셀 격자로 중복을 접으며 누적한다.
 *
 * 셋을 구분하지 않으면 화면이 거짓말을 한다. 실제로 그랬다 — 집에서 매핑한
 * 누적 점군이 남아 있는 채로 활성 맵을 alm_lab 으로 바꾸니, 화면은 alm_lab 을
 * 보여주는 것처럼 보이는데 내용은 집이었다.
 *
 * 누적은 `/cloud_registered` 로만 한다. 그 토픽은 FAST-LIO 가 이미 월드
 * 좌표로 정합해 놓은 것이라 이어 붙일 수 있다. /livox/lidar 는 센서 좌표계라
 * 그대로 쌓으면 전부 원점에 겹쳐 뭉갠다 — 누적 뷰는 본질적으로 SLAM 전용이다.
 *
 * 그리고 둘이 동시에 오면 live 는 registered 를 따른다. 두 토픽이 같은 버퍼를
 * 놓고 싸우면 화면이 서로 다른 좌표계 사이에서 깜빡인다.
 *
 * ── 왜 링버퍼가 아니라 복셀인가 ───────────────────────────────────────
 *
 * 20,000 pts/frame x 10 Hz = 200,000 pts/s. 단순 링버퍼면 200만점을 잡아도
 * 10초치다. 복셀로 접으면 **같은 자리를 다시 지나가도 점이 안 늘어난다** —
 * 실내 한 층이면 대개 수십만점에서 수렴한다.
 */
import {
  BufferAttribute,
  BufferGeometry,
  Color,
  GridHelper,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Scene,
  WebGLRenderer,
} from 'three';

const DATATYPE_FLOAT32 = 7;
const DATATYPE_FLOAT64 = 8;

/** PointCloud2 → {positions: Float32Array, count}. 실패하면 null. */
function decodePointCloud2(msg, maxPoints) {
  const fields = {};
  for (const field of msg.fields) fields[field.name] = field;
  const fx = fields.x; const fy = fields.y; const fz = fields.z;
  if (!fx || !fy || !fz) return null;
  if (fx.datatype !== DATATYPE_FLOAT32 && fx.datatype !== DATATYPE_FLOAT64) return null;

  const total = msg.width * msg.height;
  if (!total) return null;

  const bytes = msg.data;
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !msg.is_bigendian;
  const step = msg.point_step;
  const float64 = fx.datatype === DATATYPE_FLOAT64;
  const read = float64
    ? (offset) => view.getFloat64(offset, littleEndian)
    : (offset) => view.getFloat32(offset, littleEndian);

  // 너무 많으면 균등 간격으로 솎는다. 무작위로 뽑으면 프레임마다 튄다.
  const stride = Math.max(1, Math.ceil(total / maxPoints));
  const count = Math.floor((total + stride - 1) / stride);
  const positions = new Float32Array(count * 3);

  let written = 0;
  for (let i = 0; i < total; i += stride) {
    const base = i * step;
    if (base + step > bytes.byteLength) break;
    const x = read(base + fx.offset);
    const y = read(base + fy.offset);
    const z = read(base + fz.offset);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    // ROS(x 전방, y 좌, z 상) → three.js(x 우, y 상, z 후방)
    positions[written * 3] = -y;
    positions[written * 3 + 1] = z;
    positions[written * 3 + 2] = -x;
    written += 1;
  }
  return { positions, count: written, total };
}

export class Renderer3D {
  /**
   * @param {HTMLCanvasElement} canvas 목업이 쓰던 #pointCloudCanvas 를 그대로 받는다
   *   (라이브에서는 app.js 가 Canvas 2D 루프를 돌리지 않으므로 컨텍스트가 비어 있다)
   */
  constructor(canvas, {
    maxPoints = 120000,
    maxAccumulated = 600000,
    maxPrior = 400000,
    voxelSize = 0.15,
  } = {}) {
    this.maxPrior = maxPrior;
    this.priorCount = 0;
    this.priorMapName = '';
    this.canvas = canvas;
    this.maxPoints = maxPoints;
    this.maxAccumulated = maxAccumulated;
    this.voxelSize = voxelSize;
    this.running = false;
    this.lastFrame = { points: 0, at: 0 };
    this.pointsPerSecond = NaN;
    // /cloud_registered 가 마지막으로 온 시각. 이게 최근이면 SLAM 이 도는
    // 중이므로 raw /livox/lidar 는 무시한다 (좌표계가 다르다).
    this.lastRegisteredAt = 0;
    this.accumulatedCount = 0;
    this.accumulationSaturated = false;
    this._voxels = new Map();

    this.renderer = new WebGLRenderer({ canvas, antialias: false, alpha: false });
    this.renderer.setClearColor(new Color('#0A0C0F'), 1);

    this.scene = new Scene();
    const grid = new GridHelper(40, 40, 0x2a3340, 0x161c24);
    this.scene.add(grid);

    this.camera = new PerspectiveCamera(55, 1, 0.1, 500);

    // ---- 저장된 맵 (가장 아래, 가장 어두운 층) ----
    this.priorGeometry = new BufferGeometry();
    this.priorGeometry.setAttribute('position',
      new BufferAttribute(new Float32Array(maxPrior * 3), 3));
    this.priorGeometry.setDrawRange(0, 0);
    // 색 선택 기준: 배경(#0A0C0F)에서 형태가 읽히되, 실시간 레이어(파랑 계열)와
    // 색상으로 구분돼야 한다. 그래서 중성 회색을 쓰고 밝기로만 층을 나눈다.
    //   저장된 맵 #BCC8D6 (밝은 중성 회색) / 누적 #4A6B9A / 현재 스캔 #82AEFF
    // 저장맵이 가장 밝지만 색상이 무채색이라, 파랑 계열인 실시간 두 레이어와
    // 겹쳐도 서로 묻히지 않는다. 밝기가 아니라 **색상**이 층을 가른다.
    // 점 크기도 저장맵이 가장 크다 — 33,000점이 한 층에 흩어지면 밀도가 낮아
    // 작게 찍으면 아무것도 안 보인다.
    this.priorPoints = new Points(this.priorGeometry, new PointsMaterial({
      color: new Color('#BCC8D6'), size: 0.055, sizeAttenuation: true,
    }));
    this.priorPoints.frustumCulled = false;
    this.scene.add(this.priorPoints);

    // ---- 누적 맵 (아래에 깔리는 어두운 층) ----
    this.accumGeometry = new BufferGeometry();
    this.accumGeometry.setAttribute('position',
      new BufferAttribute(new Float32Array(maxAccumulated * 3), 3));
    this.accumGeometry.setDrawRange(0, 0);
    this.accumPoints = new Points(this.accumGeometry, new PointsMaterial({
      color: new Color('#4A6B9A'), size: 0.035, sizeAttenuation: true,
    }));
    // 버퍼가 계속 자라므로 경계구가 늘 낡는다. 낡은 경계구로 컬링하면 점군이
    // 통째로 사라지는 순간이 생긴다. 어차피 화면에 객체가 둘뿐이라 컬링해서
    // 얻을 것도 없다 — 아예 끈다.
    this.accumPoints.frustumCulled = false;
    this.scene.add(this.accumPoints);

    // ---- 현재 스캔 (위에 얹히는 밝은 층) ----
    this.geometry = new BufferGeometry();
    this.geometry.setAttribute('position',
      new BufferAttribute(new Float32Array(maxPoints * 3), 3));
    this.geometry.setDrawRange(0, 0);
    this.points = new Points(this.geometry, new PointsMaterial({
      color: new Color('#82AEFF'), size: 0.045, sizeAttenuation: true,
    }));
    this.points.frustumCulled = false;
    this.scene.add(this.points);

    // 궤도 카메라 상태 (three 의 OrbitControls 를 쓰지 않는다 — examples/ 를
    // 끌어오면 번들이 커지고, 여기 필요한 건 회전·줌·탑다운 세 가지뿐이다)
    this.view = { azimuth: -Math.PI / 4, elevation: 0.62, distance: 22 };
    this.defaultView = { ...this.view };
    this._bindPointer();
    this._applyCamera();

    this._loop = this._loop.bind(this);
  }

  _bindPointer() {
    let dragging = false;
    let lastX = 0;
    let lastY = 0;

    this.canvas.addEventListener('pointerdown', (event) => {
      dragging = true; lastX = event.clientX; lastY = event.clientY;
      this.canvas.setPointerCapture(event.pointerId);
    });
    this.canvas.addEventListener('pointerup', () => { dragging = false; });
    this.canvas.addEventListener('pointercancel', () => { dragging = false; });
    this.canvas.addEventListener('pointermove', (event) => {
      if (!dragging) return;
      this.view.azimuth -= (event.clientX - lastX) * 0.006;
      this.view.elevation = Math.max(
        0.05, Math.min(1.5, this.view.elevation + (event.clientY - lastY) * 0.004));
      lastX = event.clientX; lastY = event.clientY;
      this._applyCamera();
    });
    this.canvas.addEventListener('wheel', (event) => {
      event.preventDefault();
      this.view.distance = Math.max(3, Math.min(120,
        this.view.distance * (event.deltaY > 0 ? 1.1 : 0.9)));
      this._applyCamera();
    }, { passive: false });
  }

  _applyCamera() {
    const { azimuth, elevation, distance } = this.view;
    this.camera.position.set(
      distance * Math.cos(elevation) * Math.sin(azimuth),
      distance * Math.sin(elevation),
      distance * Math.cos(elevation) * Math.cos(azimuth));
    this.camera.lookAt(0, 0, 0);
  }

  resetView() {
    this.view = { ...this.defaultView };
    this._applyCamera();
  }

  topView() {
    this.view = { azimuth: 0, elevation: 1.5, distance: 30 };
    this._applyCamera();
  }

  /**
   * 센서 원본(/livox/lidar). 현재 스캔만 갱신하고 누적하지 않는다.
   *
   * SLAM 이 도는 동안에는 무시한다 — /cloud_registered 와 좌표계가 달라
   * 같은 버퍼에 번갈아 쓰면 화면이 두 좌표계 사이에서 깜빡인다.
   */
  onLiveCloud(msg) {
    if (performance.now() - this.lastRegisteredAt < 2000) return;
    this._drawLive(msg);
  }

  /** 정합된 스캔(/cloud_registered). 현재 스캔 + 누적 맵 양쪽에 반영한다. */
  onRegisteredCloud(msg) {
    const decoded = this._drawLive(msg);
    if (!decoded) return;
    // ⚠ 성공했을 때만 '정합 데이터가 온다'고 표시한다.
    //
    // 예전에는 이 줄이 맨 위에 있었다. FAST-LIO 는 기동 직후 IMU 초기화가 끝나기
    // 전까지 **빈 점군(width=0)** 을 10 Hz 로 내보내는데, 그러면
    //   ① decode 가 null → lastFrame 갱신 안 됨
    //   ② 그런데 lastRegisteredAt 은 갱신됨 → onLiveCloud 가 /livox/lidar 를 계속 무시
    // 두 개가 겹쳐 2초 뒤 hasContent() 가 false 가 되고, **SLAM 을 시작한 순간
    // placeholder 가 튀어나온다.** 정확히 그 증상을 쫓다 찾았다.
    this.lastRegisteredAt = performance.now();
    this._accumulate(decoded);
  }

  _drawLive(msg) {
    const decoded = decodePointCloud2(msg, this.maxPoints);
    if (!decoded) return null;

    const attribute = this.geometry.getAttribute('position');
    attribute.array.set(decoded.positions.subarray(0, decoded.count * 3));
    attribute.needsUpdate = true;
    this.geometry.setDrawRange(0, decoded.count);

    // 실제 처리량 [pts/s] — 모니터링 탭의 pointsRate 가 쓰던 난수를 대체한다
    const now = performance.now();
    if (this.lastFrame.at) {
      const dt = (now - this.lastFrame.at) / 1000;
      if (dt > 0) this.pointsPerSecond = decoded.total / dt;
    }
    this.lastFrame = { points: decoded.total, at: now };
    return decoded;
  }

  /**
   * 복셀 격자로 중복을 접으며 누적 버퍼에 덧붙인다.
   *
   * 키는 문자열이 아니라 **하나의 수**로 만든다. 초당 20만 점을 처리하는데
   * `${ix},${iy},${iz}` 를 쓰면 문자열 할당만으로 GC 가 밀린다.
   * 축당 17비트(0..131071)씩 3축 = 51비트 — float64 의 정수 정밀도(53비트)
   * 안에 안전하게 들어간다. 오프셋 65536 이면 복셀 0.15 m 기준 ±9.8 km 다.
   */
  _accumulate(decoded) {
    if (this.accumulationSaturated) return;
    const inv = 1 / this.voxelSize;
    const array = this.accumGeometry.getAttribute('position').array;
    const limit = this.maxAccumulated;
    const voxels = this._voxels;
    const positions = decoded.positions;
    let index = this.accumulatedCount;
    let added = 0;

    for (let i = 0; i < decoded.count; i += 1) {
      const x = positions[i * 3];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      const ix = Math.round(x * inv) + 65536;
      const iy = Math.round(y * inv) + 65536;
      const iz = Math.round(z * inv) + 65536;
      // 범위를 벗어난 점은 키가 겹쳐 엉뚱한 곳을 채우므로 버린다
      if (ix < 0 || ix > 131071 || iy < 0 || iy > 131071 || iz < 0 || iz > 131071) continue;
      const key = ix * 17179869184 + iy * 131072 + iz;   // 2^34, 2^17
      if (voxels.has(key)) continue;
      if (index >= limit) {
        this.accumulationSaturated = true;
        break;
      }
      voxels.set(key, index);
      array[index * 3] = x;
      array[index * 3 + 1] = y;
      array[index * 3 + 2] = z;
      index += 1;
      added += 1;
    }

    if (!added) return;
    this.accumulatedCount = index;
    this.accumGeometry.getAttribute('position').needsUpdate = true;
    this.accumGeometry.setDrawRange(0, index);
    // computeBoundingSphere() 를 부르지 않는다 — 60만 점을 10 Hz 로 훑으면
    // 그것만으로 프레임을 먹는다. 대신 frustumCulled=false 로 둬서 three 가
    // 경계구를 볼 일 자체를 없앴다 (생성자 참조).
  }

  /** 새 매핑을 시작할 때 이전 맵을 지운다. */
  resetAccumulation() {
    this._voxels.clear();
    this.accumulatedCount = 0;
    this.accumulationSaturated = false;
    this.accumGeometry.setDrawRange(0, 0);
  }

  /**
   * 활성 맵에 저장된 점군(/alm/prior_cloud). 누적하지 않고 통째로 교체한다.
   * 맵을 바꾸면 이 레이어만 바뀐다 — 그게 '맵을 골랐다'의 시각적 의미다.
   */
  onPriorCloud(msg) {
    // msg=null 은 '지워라' 라는 뜻이다 (재매핑 시작 시 app.js 가 부른다).
    const decoded = msg ? decodePointCloud2(msg, this.maxPrior) : null;
    const attribute = this.priorGeometry.getAttribute('position');
    if (!decoded) {                       // 빈 맵(cloud.pcd 없음)도 정상 상태다
      this.priorCount = 0;
      this.priorGeometry.setDrawRange(0, 0);
      return;
    }
    attribute.array.set(decoded.positions.subarray(0, decoded.count * 3));
    attribute.needsUpdate = true;
    this.priorGeometry.setDrawRange(0, decoded.count);
    this.priorCount = decoded.count;
  }

  /** 툴바 레이어 토글. */
  setLayerVisible(layer, visible) {
    if (layer === 'prior') this.priorPoints.visible = Boolean(visible);
    else if (layer === 'map') this.accumPoints.visible = Boolean(visible);
    else if (layer === 'scan') this.points.visible = Boolean(visible);
  }

  /**
   * 지금 이 뷰포트에 그릴 것이 있는가.
   *
   * placeholder 를 띄울지 정하는 데 쓴다. 저장된 맵이 없어도 실시간 스캔이나
   * 누적 점군이 흐르고 있으면 보여줄 것이 있는 것이다 — 매핑 중이 정확히
   * 그 상황이고, 그때 placeholder 로 덮으면 정작 봐야 할 것을 가린다.
   */
  hasContent() {
    if (this.accumulatedCount > 0 || this.priorCount > 0) return true;
    return this.lastFrame.at > 0 && (performance.now() - this.lastFrame.at) < 2000;
  }

  /**
   * 지금 화면이 무엇을 보여주고 있는가. 라벨용.
   * 이걸 화면에 쓰지 않으면 조작자가 저장된 맵과 실시간 누적을 구분할 수 없다.
   */
  describeSource() {
    const slam = performance.now() - this.lastRegisteredAt < 2000;
    if (slam) return { source: '실시간 SLAM', detail: `누적 ${this.accumulatedCount.toLocaleString()}점` };
    if (this.accumulatedCount > 0) {
      return { source: 'SLAM 종료됨', detail: `누적 ${this.accumulatedCount.toLocaleString()}점 유지 중` };
    }
    if (this.lastFrame.at) return { source: '센서 원본', detail: `${this.lastFrame.points.toLocaleString()} pts/frame` };
    return { source: '저장된 맵', detail: this.priorCount ? `${this.priorCount.toLocaleString()}점` : '점군 없음' };
  }

  /** 탭이 보일 때만 돈다. 숨겨지면 즉시 멈춘다. */
  setActive(active) {
    if (active === this.running) return;
    this.running = active;
    if (active) requestAnimationFrame(this._loop);
  }

  _loop() {
    if (!this.running) return;
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    if (width > 0 && height > 0) {
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      if (this.canvas.width !== Math.round(width * dpr)
          || this.canvas.height !== Math.round(height * dpr)) {
        this.renderer.setPixelRatio(dpr);
        this.renderer.setSize(width, height, false);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
      }
      this.renderer.render(this.scene, this.camera);
    }
    requestAnimationFrame(this._loop);
  }
}
