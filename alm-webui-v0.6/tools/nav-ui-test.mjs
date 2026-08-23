/**
 * app.js 의 자율주행 상태 기계를 DOM 없이 돌린다.
 *
 * 확인하려는 것은 '서버 상태 → 버튼/라벨' 사상이다. 브라우저를 띄우지 않고도
 * 이건 전부 결정론적으로 검사할 수 있다.
 */
import fs from 'node:fs';

// ── 최소 DOM 스텁 ──────────────────────────────────────────────────────
const nodes = new Map();
function makeNode(id) {
  const cls = new Set();
  return {
    id,
    _text: '', _title: '', disabled: false, checked: false, value: '',
    dataset: {}, innerHTML: '',
    style: { setProperty() {} },
    classList: {
      add: (c) => cls.add(c), remove: (c) => cls.delete(c),
      toggle: (c, on) => (on ? cls.add(c) : cls.delete(c)),
      contains: (c) => cls.has(c), _set: cls,
    },
    set className(v) { cls.clear(); for (const c of String(v).split(/\s+/)) if (c) cls.add(c); },
    get className() { return [...cls].join(' '); },
    set textContent(v) { this._text = v; }, get textContent() { return this._text; },
    set title(v) { this._title = v; }, get title() { return this._title; },
    addEventListener() {}, removeEventListener() {}, appendChild() {}, replaceChildren() {},
    getBoundingClientRect: () => ({ width: 900, height: 620, left: 0, top: 0 }),
    createSVGPoint: () => ({ matrixTransform: () => ({ x: 0, y: 0 }) }),
    closest: () => null, setAttribute() {}, getAttribute: () => null, focus() {}, querySelector: () => null,
  };
}
const $ = (sel) => {
  if (!nodes.has(sel)) nodes.set(sel, makeNode(sel));
  return nodes.get(sel);
};
globalThis.document = {
  readyState: 'complete',
  querySelector: $, querySelectorAll: () => [],
  addEventListener() {}, createElement: () => makeNode('new'),
  createElementNS: () => makeNode('ns'), documentElement: makeNode('html'), body: makeNode('body'),
  hidden: false,
};
globalThis.window = globalThis;
globalThis.location = { search: '', hostname: 'localhost', protocol: 'http:' };
globalThis.sessionStorage = { getItem: () => null, setItem() {}, removeItem() {} };
globalThis.localStorage = globalThis.sessionStorage;
globalThis.requestAnimationFrame = () => 0;
globalThis.confirm = () => true;
globalThis.ALM_LIVE = true;                 // 목업 루프를 띄우지 않는다

// ── app.js 적재 ────────────────────────────────────────────────────────
const source = fs.readFileSync(new URL('../assets/app.js', import.meta.url), 'utf8');
new Function(source)();
const alm = globalThis.ALM;
if (!alm) throw new Error('window.ALM 이 없다');

// 기본값 세팅
$('#repeatCount').value = '1';
$('#loopMission').checked = false;

let fails = [];
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'OK ' : 'FAIL'}] ${label}${detail ? ' — ' + detail : ''}`);
  if (!ok) fails.push(label);
};

const mission = (over = {}) => ({
  state: 'idle', kind: '', index: 0, total: 0, message: '',
  distance_remaining_m: null, eta_sec: null, recoveries: 0,
  distance_estimate_m: null, missed: [], action_ready: true, tf_ready: true,
  ...over,
});
const apply = (over = {}, stack = { process: { running: true }, nodes: [] }) =>
  alm.applyNavStatus({ ...stack, mission: mission(over) });

const start = () => ({ label: $('#startNavigation').textContent, disabled: $('#startNavigation').disabled,
  title: $('#startNavigation').title });

console.log('\n① 버튼이 상태를 정직하게 반영하는가');
alm.state.hasControl = false;
apply();
check('제어권 없으면 비활성', start().disabled, start().title);

alm.state.hasControl = true;
alm.applyNavStatus({ process: { running: false }, nodes: [], mission: mission({ action_ready: false, tf_ready: false }) });
check('스택 미기동 → 기동 버튼', start().label === '자율주행 기동' && !start().disabled, start().label);

alm.state.localization.running = true;
alm.applyNavStatus({ process: { running: false }, nodes: [], mission: mission({ action_ready: false, tf_ready: false }) });
check('측위가 따로 떠 있으면 거부', start().disabled, start().title.slice(0, 40));
alm.state.localization.running = false;

apply({ action_ready: false, tf_ready: false });
check('기동 직후 액션 없음 → 대기', start().label === '기동 중…' && start().disabled, start().label);

apply({ action_ready: true, tf_ready: false });
check('정합 전 → 대기', start().label === '정합 대기 중…' && start().disabled, start().title.slice(0, 30));

alm.state.waypoints = [];
apply();
check('웨이포인트 없으면 비활성', start().disabled, start().title);

alm.state.waypoints = [{ id: 'a', label: 'W1', x: 1, y: 2, yaw: 0 }];
apply();
check('준비 완료 → 주행 시작 활성', start().label === '주행 시작' && !start().disabled);

console.log('\n② 진척률을 지어내지 않는가');
apply({ state: 'active', kind: 'pose', total: 1, distance_remaining_m: 10 });
const p0 = $('#missionPercent').textContent;
apply({ state: 'active', kind: 'pose', total: 1, distance_remaining_m: 5 });
const p1 = $('#missionPercent').textContent;
check('단일 목표 진척률 = 남은거리 기반', p0 === '0%' && p1 === '50%', `${p0} → ${p1}`);
check('남은거리 표시', $('#remainingDistance').textContent === '5.0 m', $('#remainingDistance').textContent);

apply({ state: 'active', kind: 'waypoints', index: 1, total: 4, distance_estimate_m: 12.5 });
check('웨이포인트 진척률 = 도달수/전체', $('#missionPercent').textContent === '25%', $('#missionPercent').textContent);
check('웨이포인트는 남은거리 없음(—)', $('#remainingDistance').textContent === '—');
check('직선 추정은 title 에만', $('#remainingDistance').title.includes('12.5'), $('#remainingDistance').title.slice(0, 45));
check('현재 목표 표시', $('#currentGoalText').textContent === '2 / 4', $('#currentGoalText').textContent);

console.log('\n③ 종료 상태');
apply({ state: 'paused', kind: 'waypoints', index: 2, total: 4, message: '일시정지' });
check('PAUSED → 재개 라벨', $('#pauseNavigation').textContent === '재개' && !$('#pauseNavigation').disabled);
apply({ state: 'failed', kind: 'pose', total: 1, message: 'abort' });
check('FAILED 칩 = warning', $('#navStateChip').classList.contains('warning'), $('#navStateChip').className);
check('FAILED 면 조작 버튼 잠김', $('#pauseNavigation').disabled);
check('스택 떠 있으면 중단은 살아있음(스택 종료용)', !$('#cancelNavigation').disabled,
  $('#cancelNavigation').title);
apply({ state: 'succeeded', kind: 'waypoints', index: 4, total: 4 });
check('성공 → 100%', $('#missionPercent').textContent === '100%');
check('성공 칩 = success', $('#navStateChip').classList.contains('success'));

console.log('\n④ 시스템 상태 전이');
apply({ state: 'active', kind: 'pose', total: 1 });
check('주행 중 systemState=NAVIGATING', alm.state.systemState === 'NAVIGATING', alm.state.systemState);
apply({ state: 'idle' });
check('미션 끝나면 IDLE 복귀', alm.state.systemState === 'IDLE', alm.state.systemState);

console.log(`\n${fails.length ? '실패: ' + fails.join(', ') : '전부 통과'}`);
process.exit(fails.length ? 1 : 0);
