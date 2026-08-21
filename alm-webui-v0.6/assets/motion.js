/* ════════════════════════════════════════════════════════════
   ALM WebUI v0.6 — motion.js

   app.js의 로직을 건드리지 않고 시그니처 연출만 덧입히는 애드온.
   app.js보다 나중에 로드되어야 한다.

   1. 길게 눌러 비상정지 — 0.9초 홀드 + 링 게이지
   2. 값 변경 감지 — 숫자가 바뀔 때만 미세하게 굴러 들어온다
   3. E-STOP 상태 반영 — 화면 파동 + body 클래스
   ════════════════════════════════════════════════════════════ */
(() => {
  'use strict';

  const $ = (s) => document.querySelector(s);
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* ────────────────────────────────────────────────────────
     1. 길게 눌러 비상정지

     app.js는 #globalEstop의 click에 triggerEstop을 바인딩한다.
     여기서는 그 click을 캡처 단계에서 가로채, 0.9초를 채운
     홀드에서 발생한 click만 통과시킨다. app.js는 수정하지 않는다.
     ──────────────────────────────────────────────────────── */
  const HOLD_MS = 900;
  const btn = $('#globalEstop');
  const ring = btn && btn.querySelector('.ring-fg');
  const RING_LEN = 132;

  let raf = null;
  let startedAt = 0;
  let armed = false;

  function paint(progress) {
    if (ring) ring.style.strokeDashoffset = String(RING_LEN * (1 - progress));
  }

  function step() {
    const p = Math.min(1, (performance.now() - startedAt) / HOLD_MS);
    paint(p);
    if (p >= 1) {
      cancelHold();
      armed = true;
      btn.click();          // app.js의 triggerEstop이 여기서 실행된다
      armed = false;
      return;
    }
    raf = requestAnimationFrame(step);
  }

  function beginHold() {
    if (!btn || btn.disabled || raf) return;
    if (reduced) { armed = true; btn.click(); armed = false; return; }
    startedAt = performance.now();
    raf = requestAnimationFrame(step);
  }

  function cancelHold() {
    if (raf) cancelAnimationFrame(raf);
    raf = null;
    paint(0);
  }

  if (btn) {
    // 홀드로 완성되지 않은 click은 캡처 단계에서 차단한다.
    // 캡처 리스너를 document에 두어야 대상 요소의 리스너보다 먼저 실행된다.
    document.addEventListener('click', (e) => {
      if (!e.target.closest || !e.target.closest('#globalEstop')) return;
      if (armed) return;
      e.stopPropagation();
      e.preventDefault();
    }, true);

    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      btn.setPointerCapture(e.pointerId);
      beginHold();
    });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach((ev) =>
      btn.addEventListener(ev, cancelHold));

    // 키보드로도 동일하게 — 접근성이자 안전 요건
    btn.addEventListener('keydown', (e) => {
      if (e.key !== 'Enter' && e.key !== ' ') return;
      e.preventDefault();
      beginHold();
    });
    btn.addEventListener('keyup', cancelHold);
    btn.addEventListener('blur', cancelHold);
  }

  /* ────────────────────────────────────────────────────────
     2. 값 변경 감지

     DOM을 다시 쓰지 않고 읽기만 한다. app.js가 textContent를
     갱신하면 다음 프레임에 감지해 클래스만 붙인다.
     ──────────────────────────────────────────────────────── */
  const WATCH = [
    'monCpu', 'monGpu', 'monRam', 'monPower', 'pointsRate',
    'mappingCpu', 'mappingRam', 'mappingGpu', 'mappingTemp',
    'pointCount', 'missionPercent', 'batteryTop', 'sidebarCpu',
    'tempCpu', 'tempGpu', 'tempSoc', 'tempTj', 'netTx', 'netRx',
  ];

  const watched = WATCH
    .map((id) => document.getElementById(id))
    .filter(Boolean)
    .map((el) => ({ el, last: el.textContent }));

  function sweep() {
    for (const item of watched) {
      const now = item.el.textContent;
      if (now === item.last) continue;
      item.last = now;
      item.el.classList.remove('tick');
      void item.el.offsetWidth;   // 애니메이션 재시작
      item.el.classList.add('tick');
    }
    requestAnimationFrame(sweep);
  }
  if (!reduced && watched.length) requestAnimationFrame(sweep);

  /* ────────────────────────────────────────────────────────
     3. E-STOP 상태 반영

     app.js는 #safetyStrip에 .estopped를 토글한다.
     그 변화만 관찰해 화면 파동과 body 클래스를 붙인다.
     ──────────────────────────────────────────────────────── */
  const strip = $('#safetyStrip');
  const flash = $('#estopFlash');

  if (strip) {
    let wasEstopped = strip.classList.contains('estopped');

    new MutationObserver(() => {
      const now = strip.classList.contains('estopped');
      if (now === wasEstopped) return;
      wasEstopped = now;

      document.body.classList.toggle('estop-on', now);

      if (now && flash && !reduced) {
        flash.classList.remove('fire');
        void flash.offsetWidth;
        flash.classList.add('fire');
      }
      if (now) cancelHold();
    }).observe(strip, { attributes: true, attributeFilter: ['class'] });
  }
})();
