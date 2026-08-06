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
  constructor(canvas, { maxPoints = 120000 } = {}) {
    this.canvas = canvas;
    this.maxPoints = maxPoints;
    this.running = false;
    this.lastFrame = { points: 0, at: 0 };
    this.pointsPerSecond = NaN;

    this.renderer = new WebGLRenderer({ canvas, antialias: false, alpha: false });
    this.renderer.setClearColor(new Color('#0A0C0F'), 1);

    this.scene = new Scene();
    const grid = new GridHelper(40, 40, 0x2a3340, 0x161c24);
    this.scene.add(grid);

    this.camera = new PerspectiveCamera(55, 1, 0.1, 500);

    this.geometry = new BufferGeometry();
    this.geometry.setAttribute('position',
      new BufferAttribute(new Float32Array(maxPoints * 3), 3));
    this.geometry.setDrawRange(0, 0);
    this.points = new Points(this.geometry, new PointsMaterial({
      color: new Color('#82AEFF'), size: 0.045, sizeAttenuation: true,
    }));
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

  onPointCloud(msg) {
    const decoded = decodePointCloud2(msg, this.maxPoints);
    if (!decoded) return;

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
