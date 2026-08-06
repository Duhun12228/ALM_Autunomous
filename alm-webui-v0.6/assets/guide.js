/* ════════════════════════════════════════════════════════════
   ALM WebUI v0.6 — guide.js
   탭별 최초 진입 스포트라이트 가이드 투어.

   app.js의 로직은 건드리지 않고, 탭 전환과 콘솔 진입을
   관찰만 해서 동작한다. app.js보다 나중에 로드되어야 한다.

   - 탭에 처음 들어가면(이번 세션 한정) 자동으로 시작
   - HUD의 '?' 버튼으로 언제든 다시 시작 가능
   - Esc 또는 × 로 언제든 종료
   ════════════════════════════════════════════════════════════ */
(() => {
  'use strict';

  const $ = (s, r = document) => r.querySelector(s);
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const STORAGE_KEY = 'alm-guide-seen';

  const TOURS = {
    mapping: [
      { target: '#mappingProfile', title: '매핑 방식 선택', text: '방식 A/B/C 중 하나를 고릅니다. 방식마다 필요한 준비물이 다르니, 아래 설명을 먼저 확인하세요.' },
      { target: '#newMapButton', title: '새 지도 만들기', text: '완전히 새로운 이름으로 빈 지도를 만들고 바로 시작할 수 있습니다. 매핑이 진행 중일 때는 눌러도 반응하지 않습니다.' },
      { target: '#startMapping', title: 'SLAM 시작', text: '센서 켜기 → 매핑 실행 순서로 자동 진행됩니다. 이때부터 로봇 이동은 외부 조이스틱으로만 합니다.' },
      { target: '#savePcd', title: '3D 지도 저장', text: '지금까지 쌓은 3D 지도를 파일로 저장합니다. 다음 단계들을 하려면 이 버튼을 먼저 눌러야 합니다.' },
      { target: '#openPcd2Pgm', title: '2D 지도로 변환', text: '3D 지도를 위에서 내려다본 흑백 2D 지도로 바꿉니다. 자율주행이 실제로 사용하는 지도 형태입니다.' },
      { target: '#buildScDb', title: '위치 인식 데이터 생성', text: '나중에 로봇이 스스로 자기 위치를 찾을 수 있도록 장소별 모양 정보를 저장합니다. 방식 A에서는 필요 없습니다.' },
    ],
    navigation: [
      { target: '#autoLocalization', title: '초기위치 자동 탐색', text: '자율주행을 시작하려면 먼저 "로봇이 지금 지도의 어디에 있는지" 확정해야 합니다. 이 버튼을 누르면 자동으로 찾아줍니다.' },
      { target: '#addWaypointMode', title: '목적지(웨이포인트) 추가', text: '누른 뒤 지도를 클릭하면 그 자리가 목적지로 추가됩니다. 순서대로 여러 개 찍어 경로를 만들 수 있습니다.' },
      { target: '#startNavigation', title: '주행 시작', text: '초기위치가 확정되고 목적지가 1개 이상 있으면 자율주행이 시작됩니다. 조건이 안 맞으면 이유가 안내됩니다.' },
      { target: '#pauseNavigation', title: '일시정지 · 재개', text: '주행 중 잠깐 멈췄다가 남은 목적지를 그대로 이어서 진행할 수 있습니다.' },
      { target: '.layer-panel', title: '지도 레이어', text: '체크박스로 경로, 위험지도(코스트맵), 라이다 스캔 등 보고 싶은 정보만 켜고 끌 수 있습니다.' },
    ],
    manual: [
      { target: '#enterManual', title: '모터 활성화', text: '수동 조종을 시작하려면 먼저 이 버튼으로 모터를 켭니다. 안전 확인 창이 한 번 뜹니다.' },
      { target: '#manualModeSelector', title: '주행 모드', text: 'Normal(보통 주행) · Spin(제자리 회전) · Crab(옆으로 이동, 현재 비활성) 중 고를 수 있습니다.' },
      { target: '#speedMultiplier', title: '속도 배율', text: '25~100% 중 선택한 배율만큼 이동 속도가 정해집니다. 처음에는 낮은 배율을 권장합니다.' },
      { target: '.deadman-pad', title: '조종 버튼 (데드맨 방식)', text: '버튼을 누르고 있는 동안에만 로봇이 움직이고, 손을 떼면 즉시 멈추는 안전한 방식입니다.' },
      { target: '.twist-readout', title: '실시간 속도 확인', text: '지금 보내고 있는 속도 명령값을 숫자와 막대그래프로 바로 확인할 수 있습니다.' },
    ],
    monitoring: [
      { target: '.readouts', title: '온보드 자원', text: '로봇에 탑재된 컴퓨터의 연산 사용량과 전력 소비를 한눈에 봅니다.' },
      { target: '.wave', title: '실시간 그래프', text: '방금 본 수치들이 시간에 따라 어떻게 변했는지 꺾은선 그래프로 보여줍니다.' },
      { target: '.interlocks', title: '안전 인터록', text: '비상정지, 명령 끊김 감지 등 6가지 안전장치가 모두 정상인지 한 번에 확인합니다.' },
      { target: '#exportSnapshot', title: '스냅샷 내보내기', text: '지금 이 순간의 모든 상태를 파일 하나로 저장해 기록하거나 공유할 수 있습니다.' },
    ],
  };

  function getSeen() {
    try { return JSON.parse(sessionStorage.getItem(STORAGE_KEY) || '{}'); }
    catch { return {}; }
  }
  function markSeen(tab) {
    try {
      const seen = getSeen();
      seen[tab] = true;
      sessionStorage.setItem(STORAGE_KEY, JSON.stringify(seen));
    } catch { /* storage unavailable — tour just won't be remembered next time */ }
  }

  const overlay = $('#tourOverlay');
  if (!overlay) return;
  const highlight = $('#tourHighlight');
  const tooltip = $('#tourTooltip');
  const stepCountEl = $('#tourStepCount');
  const titleEl = $('#tourTitle');
  const textEl = $('#tourText');
  const prevBtn = $('#tourPrev');
  const nextBtn = $('#tourNext');
  const closeBtn = $('#tourClose');

  let activeTour = null;
  let stepIndex = 0;
  let currentTab = 'mapping';

  function visibleTarget(selector) {
    const el = $(selector);
    if (!el) return null;
    const rect = el.getBoundingClientRect();
    return rect.width && rect.height ? el : null;
  }

  function positionFor(el) {
    const rect = el.getBoundingClientRect();
    const pad = 6;
    highlight.style.top = `${rect.top - pad}px`;
    highlight.style.left = `${rect.left - pad}px`;
    highlight.style.width = `${rect.width + pad * 2}px`;
    highlight.style.height = `${rect.height + pad * 2}px`;

    const ttRect = tooltip.getBoundingClientRect();
    const spaceBelow = window.innerHeight - rect.bottom;
    const placeBelow = spaceBelow > ttRect.height + 24 || spaceBelow > rect.top;
    const top = placeBelow ? rect.bottom + pad + 14 : rect.top - pad - 14 - ttRect.height;
    let left = rect.left + rect.width / 2 - ttRect.width / 2;
    left = Math.max(12, Math.min(left, window.innerWidth - ttRect.width - 12));
    tooltip.style.top = `${Math.max(12, Math.min(top, window.innerHeight - ttRect.height - 12))}px`;
    tooltip.style.left = `${left}px`;
  }

  function showStep() {
    const steps = TOURS[activeTour];
    let step = steps[stepIndex];
    let el = step ? visibleTarget(step.target) : null;
    while (step && !el && stepIndex < steps.length - 1) {
      stepIndex += 1;
      step = steps[stepIndex];
      el = step ? visibleTarget(step.target) : null;
    }
    if (!step || !el) { closeTour(); return; }

    stepCountEl.textContent = `${stepIndex + 1} / ${steps.length}`;
    titleEl.textContent = step.title;
    textEl.textContent = step.text;
    prevBtn.disabled = stepIndex === 0;
    nextBtn.textContent = stepIndex === steps.length - 1 ? '완료' : '다음 →';

    positionFor(el);
    el.scrollIntoView({ block: 'center', behavior: reduced ? 'auto' : 'smooth' });
    setTimeout(() => positionFor(el), reduced ? 0 : 320);
  }

  function startTour(tab) {
    if (!TOURS[tab]) return;
    activeTour = tab;
    stepIndex = 0;
    overlay.classList.remove('hidden');
    showStep();
  }

  function closeTour() {
    overlay.classList.add('hidden');
    if (activeTour) markSeen(activeTour);
    activeTour = null;
  }

  nextBtn.addEventListener('click', () => {
    const steps = TOURS[activeTour];
    if (stepIndex >= steps.length - 1) { closeTour(); return; }
    stepIndex += 1;
    showStep();
  });
  prevBtn.addEventListener('click', () => {
    if (stepIndex === 0) return;
    stepIndex -= 1;
    showStep();
  });
  closeBtn.addEventListener('click', closeTour);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && activeTour) closeTour();
  });
  window.addEventListener('resize', () => {
    if (!activeTour) return;
    const step = TOURS[activeTour][stepIndex];
    const el = step && visibleTarget(step.target);
    if (el) positionFor(el);
  });

  $('#openGuide')?.addEventListener('click', () => startTour(currentTab));

  /* ── 현재 활성 탭 추적 (app.js의 switchTab을 관찰만 함) ── */
  const tabPanels = [...document.querySelectorAll('.tab-panel')];
  function readCurrentTab() {
    const active = tabPanels.find((panel) => panel.classList.contains('active'));
    if (active) currentTab = active.id.replace('tab-', '');
  }
  readCurrentTab();

  new MutationObserver(() => {
    const before = currentTab;
    readCurrentTab();
    if (currentTab !== before && document.body.classList.contains('entered') && !getSeen()[currentTab]) {
      startTour(currentTab);
    }
  }).observe($('main.stage') || document.body, { subtree: true, attributes: true, attributeFilter: ['class'] });

  /* ── 0번 화면(Cold Start)을 통과해 콘솔에 진입하면 첫 탭 가이드 자동 시작 ── */
  function tryAutoStartOnEntry(observerToStop) {
    if (!document.body.classList.contains('entered')) return;
    if (observerToStop) observerToStop.disconnect();
    if (!activeTour && !getSeen()[currentTab]) startTour(currentTab);
  }
  if (document.body.classList.contains('entered')) {
    tryAutoStartOnEntry(null);
  } else {
    const entryObserver = new MutationObserver(() => tryAutoStartOnEntry(entryObserver));
    entryObserver.observe(document.body, { attributes: true, attributeFilter: ['class'] });
  }
})();
