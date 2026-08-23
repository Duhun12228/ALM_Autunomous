/**
 * 지도 마커 시험 — 로봇 크기 축척과 목표 헤딩 드래그.
 *
 * 두 가지를 브라우저 없이 확인한다.
 *   ① 로봇 마커가 **맵 축척을 따라간다** (미터로 그리고 px/m 를 곱하는가)
 *   ② 끌어서 지정한 헤딩이 ROS 규약(반시계 양수)으로 저장되는가
 *
 * ②가 특히 손으로 검산하기 나쁘다. 화면은 y 가 아래로 증가하고 ROS 는 위로
 * 증가하므로, 각도를 어느 좌표계에서 재느냐로 부호가 통째로 뒤집힌다.
 * '위로 끌었는데 로봇이 아래를 본다' 는 눈으로 보기 전에는 모른다.
 */
import fs from 'node:fs';

// ── DOM 스텁 (listener 를 기록해 실제 핸들러를 부를 수 있게 한다) ──────────
const nodes = new Map();
function makeNode(id) {
  const cls = new Set();
  const listeners = new Map();
  return {
    id, _text: '', _title: '', disabled: false, checked: false, value: '',
    dataset: {}, innerHTML: '', style: { setProperty() {} },
    classList: {
      add: (c) => cls.add(c), remove: (c) => cls.delete(c),
      toggle: (c, on) => (on ? cls.add(c) : cls.delete(c)),
      contains: (c) => cls.has(c),
    },
    set className(v) { cls.clear(); for (const c of String(v).split(/\s+/)) if (c) cls.add(c); },
    get className() { return [...cls].join(' '); },
    set textContent(v) { this._text = v; }, get textContent() { return this._text; },
    set title(v) { this._title = v; }, get title() { return this._title; },
    addEventListener(type, fn) {
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(fn);
    },
    removeEventListener() {},
    /** 시험용 — 등록된 핸들러를 순서대로 부른다. */
    fire(type, event = {}) {
      for (const fn of listeners.get(type) || []) fn({ ...event, currentTarget: this, preventDefault() {} });
    },
    _attrs: {},
    setAttribute(k, v) { this._attrs[k] = v; }, getAttribute(k) { return this._attrs[k] ?? null; },
    appendChild() {}, replaceChildren() {}, closest: () => null, focus() {}, querySelector: () => null,
    getBoundingClientRect: () => ({ width: 900, height: 620, left: 0, top: 0 }),
    // svgPoint(): 화면좌표를 그대로 SVG 좌표로 쓴다 (항등변환)
    createSVGPoint: () => ({ x: 0, y: 0, matrixTransform() { return { x: this.x, y: this.y }; } }),
    getScreenCTM: () => ({ inverse: () => ({}) }),
    setPointerCapture() {}, releasePointerCapture() {},
    // Map2D 가 OccupancyGrid 를 구울 때 쓴다
    width: 0, height: 0,
    getContext: () => ({
      createImageData: (w, h) => ({ data: new Uint8ClampedArray(w * h * 4) }),
      putImageData() {},
    }),
    toDataURL: () => 'data:image/png;base64,',
  };
}
const $ = (sel) => {
  if (!nodes.has(sel)) nodes.set(sel, makeNode(sel));
  return nodes.get(sel);
};
const docListeners = new Map();
globalThis.document = {
  readyState: 'loading',
  querySelector: $, querySelectorAll: () => [],
  addEventListener(type, fn) {
    if (!docListeners.has(type)) docListeners.set(type, []);
    docListeners.get(type).push(fn);
  },
  createElement: () => makeNode('canvas'),
  createElementNS: () => makeNode('ns'),
  documentElement: makeNode('html'), body: makeNode('body'), hidden: false,
};
globalThis.window = globalThis;
globalThis.addEventListener = () => {};   // app.js 가 window 에도 몇 개 건다
globalThis.removeEventListener = () => {};
globalThis.location = { search: '', hostname: 'localhost', protocol: 'http:' };
globalThis.sessionStorage = { getItem: () => null, setItem() {}, removeItem() {} };
globalThis.localStorage = globalThis.sessionStorage;
globalThis.requestAnimationFrame = () => 0;
globalThis.confirm = () => true;
globalThis.ALM_LIVE = true;

new Function(fs.readFileSync(new URL('../assets/app.js', import.meta.url), 'utf8'))();
const alm = globalThis.ALM;
$('#defaultTolerance').value = '0.25';
$('#repeatCount').value = '1';
// bind() 를 돌려 포인터 핸들러를 실제로 등록시킨다
for (const fn of docListeners.get('DOMContentLoaded') || []) fn({});

const { Map2D } = await import('../src/render/map2d.js');

let fails = [];
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'OK ' : 'FAIL'}] ${label}${detail ? ' — ' + detail : ''}`);
  if (!ok) fails.push(label);
};
const near = (a, b, tol = 0.05) => Math.abs(a - b) <= tol;

// nav2.yaml 의 footprint: x[-0.65, 0.72] y[±0.53]
const ROBOT_LENGTH_M = 1.37;

const grid = (width, height, resolution = 0.05) => ({
  info: { width, height, resolution, origin: { position: { x: 0, y: 0 } } },
  data: new Int8Array(width * height),
});

const map2d = new Map2D();

console.log('\n① 로봇 마커가 맵 축척을 따라가는가');
map2d.onOccupancyGrid(grid(1014, 671));              // alm_lab 실측 크기
const big = map2d.robotScale();
check('실측맵 축척 = 0.88757/0.05', near(big, 17.75, 0.02), `${big.toFixed(2)} px/m`);
check('화면상 차체 길이가 실제와 일치',
  near(ROBOT_LENGTH_M * big, 24.3, 0.2), `${(ROBOT_LENGTH_M * big).toFixed(1)} SVG단위 = 1.37 m`);

map2d.setRobotPose({ x: 10, y: 5, yaw: 0 });
const transform = $('#robotLayer').getAttribute('transform');
check('transform 에 scale 이 실린다', /scale\(17\.7\d+\)/.test(transform), transform);

map2d.onOccupancyGrid(grid(200, 140));               // 작은 맵 = 더 크게 보여야
const small = map2d.robotScale();
check('작은 맵에서는 축척이 커진다', small > big, `${small.toFixed(2)} > ${big.toFixed(2)} px/m`);

map2d.onOccupancyGrid(grid(6000, 4000));             // 아주 넓은 맵
const huge = map2d.robotScale();
check('넓은 맵에서 하한(10)으로 잡힌다', near(huge, 10, 0.001), `${huge.toFixed(2)} px/m`);
check('하한이 없었다면 안 보였을 것', 900 / 6000 / 0.05 < 10,
  `무보정 ${(900 / 6000 / 0.05).toFixed(2)} px/m → 차체 ${(ROBOT_LENGTH_M * 900 / 6000 / 0.05).toFixed(1)} SVG단위`);

console.log('\n② 끌어서 지정한 헤딩이 ROS 규약을 따르는가');
map2d.onOccupancyGrid(grid(1014, 671));              // 변환을 실측맵으로 되돌린다
alm.state.hasControl = true;
alm.state.addWaypoint = true;
alm.state.waypoints = [];
const map = $('#navigationMap');

const drag = (from, to) => {
  alm.state.waypoints = [];
  map.fire('pointerdown', { clientX: from[0], clientY: from[1], pointerId: 1 });
  map.fire('pointermove', { clientX: to[0], clientY: to[1], pointerId: 1 });
  map.fire('pointerup', { clientX: to[0], clientY: to[1], pointerId: 1 });
  return alm.state.waypoints[0];
};

// 화면에서 **위로** 끌면 ROS 의 +y(반시계 90°) 여야 한다.
const up = drag([100, 300], [100, 200]);
check('위로 끌기 → yaw +90°', near(up.yaw, 90, 0.5), `${up.yaw}°`);
const right = drag([100, 300], [200, 300]);
check('오른쪽으로 끌기 → yaw 0°', near(right.yaw, 0, 0.5), `${right.yaw}°`);
const down = drag([100, 300], [100, 400]);
check('아래로 끌기 → yaw -90°', near(down.yaw, -90, 0.5), `${down.yaw}°`);
const left = drag([100, 300], [20, 300]);
check('왼쪽으로 끌기 → yaw 180°', near(Math.abs(left.yaw), 180, 0.5), `${left.yaw}°`);

const tap = drag([100, 300], [102, 302]);
check('안 끌면(손떨림) yaw 0 — 무작위 방향 방지', tap.yaw === 0, `${tap.yaw}°`);

console.log('\n③ 위치는 지도 좌표에서 다시 계산하는가');
const placed = drag([100, 300], [160, 300]);
check('클릭 지점이 미터로 저장된다', near(placed.x, 5.633, 0.01) && near(placed.y, 17.339, 0.01),
  `x=${placed.x.toFixed(3)} y=${placed.y.toFixed(3)}`);
const beforeHtml = $('#waypointLayer').innerHTML;
map2d.onOccupancyGrid(grid(500, 400));               // 축척이 바뀌면
alm.renderWaypoints();
const afterHtml = $('#waypointLayer').innerHTML;
const svgX = (html) => Number(/translate\(([-\d.]+)/.exec(html)?.[1]);
check('축척이 바뀌면 핀 화면좌표도 따라 바뀐다', svgX(beforeHtml) !== svgX(afterHtml),
  `${svgX(beforeHtml)} → ${svgX(afterHtml)} (같은 지점 x=${placed.x.toFixed(2)} m)`);
check('헤딩 화살표가 핀에 그려진다', beforeHtml.includes('rotate('), beforeHtml.slice(0, 60));

console.log(`\n${fails.length ? '실패: ' + fails.join(', ') : '전부 통과'}`);
process.exit(fails.length ? 1 : 0);
