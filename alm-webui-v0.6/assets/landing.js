/* ════════════════════════════════════════════════════════════
   ALM WebUI v0.6 — landing.js
   0번 화면(Cold Start)의 살아있는 프리뷰 + 진입/복귀 제어.

   랜딩은 콘솔 위에 얹히는 오버레이다. app.js의 탭 로직과
   독립적으로 동작하며, "진입" 시 랜딩만 걷힌다.
   미니 프리뷰는 app.js의 캔버스 감각을 축소 재현한다.
   ════════════════════════════════════════════════════════════ */
(() => {
  'use strict';
  const $ = (s, r = document) => r.querySelector(s);
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const shell = $('#coldStart');
  if (!shell) return;

  /* 세션 내 재방문 시엔 바로 콘솔로 (원하면 주석 해제)
  if (sessionStorage.getItem('alm-entered')) { enter(true); } */

  document.body.classList.add('booted');

  /* ── 진입 / 복귀 ─────────────────────────────────────── */
  function enter(instant) {
    stopLoops();
    document.body.classList.remove('booted');
    if (instant || reduced) {
      document.body.classList.add('entered');
    } else {
      shell.classList.add('dismissed');
      shell.addEventListener('animationend', () => document.body.classList.add('entered'), { once: true });
    }
    sessionStorage.setItem('alm-entered', '1');
  }

  function reopen() {
    resetReveals();
    document.body.classList.remove('entered');
    document.body.classList.add('booted');
    shell.classList.remove('dismissed');
    shell.scrollTop = 0;
    startLoops();
  }

  $('#enterConsole')?.addEventListener('click', () => enter(false));
  $('#skipBoot')?.addEventListener('click', () => enter(false));
  // HUD 브랜드를 누르면 다시 0번 화면으로
  $('#coldStartReopen')?.addEventListener('click', (e) => { e.preventDefault(); reopen(); });

  /* ── 부팅 체크 점등 (스크롤 연동, JS는 관찰만) ──────────── */
  const checks = [...document.querySelectorAll('.boot-check')];
  const sections = [...document.querySelectorAll('.cs-section[data-check]')];
  const pct = $('#bootPct');
  const fillEl = $('.boot-fill');

  function onScroll() {
    const h = shell.scrollHeight - shell.clientHeight;
    let p = h > 0 ? Math.min(1, shell.scrollTop / h) : 0;
    if (p > 0.985) p = 1;                       // 바닥은 100%로 스냅
    if (pct) pct.textContent = String(Math.round(p * 100)).padStart(3, '0') + '%';
    if (fillEl) fillEl.style.transform = 'scaleY(' + p + ')';

    const mid = shell.scrollTop + shell.clientHeight * 0.5;
    let reached = -1;
    sections.forEach((sec) => {
      if (sec.offsetTop <= mid) reached = Math.max(reached, Number(sec.dataset.check));
    });
    // 바닥 근처면 마지막 체크까지 확실히 점등
    if (p > 0.92) reached = checks.length - 1;
    checks.forEach((c, i) => c.classList.toggle('lit', i <= reached));
  }
  shell.addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  /* ── 스크롤 등장 시퀀스 ────────────────────────────────
     CSS Scroll-driven Animation 대신 IntersectionObserver를 사용해
     브라우저에 관계없이 로고와 같은 페이드 · 상승 · 블러를 재현한다. */
  const revealItems = [];

  function addRevealItem(element, delay = 0) {
    if (!element || revealItems.includes(element)) return;
    element.classList.add('reveal-item');
    element.style.setProperty('--reveal-delay', `${delay}ms`);
    revealItems.push(element);
  }

  document.querySelectorAll('.reveal').forEach((element) => {
    if (element.classList.contains('subsys')) {
      addRevealItem(element.querySelector('.subsys-copy'), 0);
      addRevealItem(element.querySelector('.subsys-demo'), 120);
      return;
    }
    const delay = element.classList.contains('d3') ? 240
      : element.classList.contains('d2') ? 160
        : element.classList.contains('d1') ? 80 : 0;
    addRevealItem(element, delay);
  });

  document.querySelectorAll('.reveal-stagger').forEach((container) => {
    [...container.children].forEach((element, index) => {
      addRevealItem(element, Math.min(index, 8) * 80);
    });
  });

  let revealObserver = null;

  function resetReveals() {
    revealItems.forEach((element) => {
      element.style.transition = 'none';
      element.classList.remove('is-visible');
    });
    void shell.offsetHeight;
    revealItems.forEach((element) => element.style.removeProperty('transition'));
  }

  if (!reduced && 'IntersectionObserver' in window) {
    shell.classList.add('reveal-ready');
    revealObserver = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) entry.target.classList.add('is-visible');
      });
    }, {
      root: shell,
      threshold: 0.12,
      rootMargin: '0px 0px -8% 0px',
    });
    revealItems.forEach((element) => revealObserver.observe(element));
  } else {
    revealItems.forEach((element) => element.classList.add('is-visible'));
  }

  /* ── 살아있는 프리뷰 ────────────────────────────────── */
  let raf = null;
  let running = false;

  function fit(canvas) {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (!r.width) return null;
    if (canvas.width !== Math.round(r.width * dpr)) {
      canvas.width = Math.round(r.width * dpr);
      canvas.height = Math.round(r.height * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w: r.width, h: r.height };
  }

  /* 배경: 전면 포인트클라우드 (app.js drawPointCloud의 축약) */
  function drawBg() {
    const c = $('#csBgCanvas');
    if (!c) return;
    const g = fit(c);
    if (!g) return;
    const { ctx, w, h } = g;
    ctx.clearRect(0, 0, w, h);
    const t = performance.now() * 0.00016;
    const cx = w * 0.5, cy = h * 0.52;
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(-0.06);
    for (let i = 0; i < 1300; i++) {
      const a = (i * 1.618 + t) % (Math.PI * 2);
      const rad = 60 + ((i * 41) % Math.min(w, h) * 0.9);
      const room = i % 5 === 0 ? 1 : 0.6;
      let x = Math.cos(a) * rad * room;
      let y = Math.sin(a) * rad * 0.4 * room;
      ctx.fillStyle = i % 13 === 0 ? 'rgba(111,168,255,.8)' : `rgba(150,175,220,${0.12 + (i % 5) * 0.05})`;
      const s = i % 13 === 0 ? 2 : 1.1;
      ctx.fillRect(x, y, s, s);
    }
    ctx.restore();
  }

  /* 미니 매핑 프리뷰 */
  function drawMiniMap() {
    const c = $('#demoMapping');
    if (!c) return;
    const g = fit(c);
    if (!g) return;
    const { ctx, w, h } = g;
    ctx.clearRect(0, 0, w, h);
    const t = performance.now() * 0.0003;
    const cx = w * 0.5, cy = h * 0.56;
    ctx.strokeStyle = 'rgba(111,168,255,.1)'; ctx.lineWidth = 1;
    for (let x = -w; x < w * 2; x += 34) { ctx.beginPath(); ctx.moveTo(cx, cy * 0.5); ctx.lineTo(x, h); ctx.stroke(); }
    ctx.save(); ctx.translate(cx, cy);
    for (let i = 0; i < 640; i++) {
      const a = (i * 1.618 + t) % (Math.PI * 2);
      const rad = 26 + ((i * 29) % 150);
      const room = i % 4 === 0 ? 1 : 0.55;
      const x = Math.cos(a) * rad * room, y = Math.sin(a) * rad * 0.4 * room;
      ctx.fillStyle = i % 11 === 0 ? 'rgba(63,239,137,.85)' : `rgba(130,174,255,${0.2 + (i % 4) * 0.08})`;
      ctx.fillRect(x, y, 1.4, 1.4);
    }
    ctx.restore();
  }

  /* 미니 모니터링 파형 */
  const wave = [];
  for (let i = 0; i < 70; i++) wave.push(30 + Math.random() * 12);
  let waveTick = 0;
  function drawMiniWave() {
    const c = $('#demoMonitor');
    if (!c) return;
    const g = fit(c);
    if (!g) return;
    const { ctx, w, h } = g;
    ctx.clearRect(0, 0, w, h);
    if (++waveTick % 3 === 0) { wave.push(30 + Math.random() * 16 + Math.sin(waveTick * 0.1) * 6); if (wave.length > 70) wave.shift(); }
    const n = wave.length;
    const px = (i) => i / (n - 1) * w;
    const py = (v) => h - (v / 100) * (h - 20) - 10;
    ctx.strokeStyle = 'rgba(255,255,255,.06)'; ctx.lineWidth = 1;
    for (let i = 1; i < 3; i++) { const y = h * i / 3; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    ctx.beginPath();
    wave.forEach((v, i) => i ? ctx.lineTo(px(i), py(v)) : ctx.moveTo(px(i), py(v)));
    ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, 'rgba(111,168,255,.22)'); grad.addColorStop(1, 'rgba(111,168,255,0)');
    ctx.fillStyle = grad; ctx.fill();
    ctx.beginPath();
    wave.forEach((v, i) => i ? ctx.lineTo(px(i), py(v)) : ctx.moveTo(px(i), py(v)));
    ctx.strokeStyle = '#6FA8FF'; ctx.lineWidth = 1.6; ctx.lineJoin = 'round'; ctx.stroke();
  }

  /* 미니 데드맨: 방향키 순환 점등 */
  let padStep = 0, padLast = 0;
  const padKeys = ['up', 'lf', 'rt', 'dn', 'st'];
  function cyclePad(now) {
    if (now - padLast < 700) return;
    padLast = now;
    document.querySelectorAll('.mini-pad .k').forEach((k) => k.classList.remove('press'));
    const key = padKeys[padStep % padKeys.length];
    const el = $('.mini-pad .k.' + key);
    if (el) el.classList.add('press');
    padStep++;
  }

  function loop(now) {
    drawBg();
    drawMiniMap();
    drawMiniWave();
    cyclePad(now || 0);
    raf = requestAnimationFrame(loop);
  }
  function startLoops() {
    if (running || reduced) { if (reduced) { drawBg(); drawMiniMap(); drawMiniWave(); } return; }
    running = true; raf = requestAnimationFrame(loop);
  }
  function stopLoops() {
    running = false;
    if (raf) cancelAnimationFrame(raf);
    raf = null;
  }

  window.addEventListener('resize', () => { if (!running) { drawBg(); drawMiniMap(); drawMiniWave(); } });
  startLoops();
})();
