/**
 * 맵이 아직 없을 때 자리를 지키는 그림.
 *
 * 인라인 SVG 다 — 외부 이미지 파일을 쓰지 않는다. 로봇 온보드에는 외부망이
 * 없으므로(웹폰트 CDN 과 같은 이유) 이미지를 URL 로 부르면 현장에서 깨진다.
 *
 * 목적은 장식이 아니라 **거짓말을 막는 것**이다. 목업 시절에는 맵이 없어도
 * 예전 화면이 그대로 남아 있어서 조작자가 "맵이 있다"고 착각했다. 여기서는
 * 없으면 없다고 말한다.
 */

const SR = 'http://www.w3.org/2000/svg';

/**
 * @param {'cloud'|'grid'} kind  없는 자산
 * @param {string} mapName       활성 맵 이름 (없으면 '—')
 */
function markup(kind, mapName) {
  const is3d = kind === 'cloud';
  const title = is3d ? '아직 이 공간을 모릅니다' : '2D 맵이 아직 없습니다';
  const detail = is3d
    ? 'SLAM을 시작하면 여기에 3D 맵이 그려집니다'
    : 'cloud.pcd 를 pcd2pgm 으로 변환하면 여기에 표시됩니다';

  return `
  <svg class="alm-ph-art" viewBox="0 0 220 150" role="img" aria-label="${title}">
    <!-- 미탐색 영역 -->
    <circle class="alm-ph-scanring" cx="110" cy="86" r="52" />
    <circle class="alm-ph-scanring d2" cx="110" cy="86" r="34" />

    <!-- 라이다 스캔 호 (천천히 회전) -->
    <g class="alm-ph-sweep" style="transform-origin:110px 86px">
      <path d="M110 86 L110 30 A56 56 0 0 1 158 58 Z" />
    </g>

    <!-- 로봇 -->
    <g class="alm-ph-bot">
      <rect x="88" y="72" width="44" height="32" rx="11" />
      <circle class="alm-ph-eye" cx="101" cy="86" r="4.2" />
      <circle class="alm-ph-eye d2" cx="119" cy="86" r="4.2" />
      <!-- 라이다 '귀' -->
      <rect class="alm-ph-lidar" x="104" y="62" width="12" height="8" rx="3" />
      <line class="alm-ph-ant" x1="110" y1="62" x2="110" y2="54" />
      <circle class="alm-ph-ping" cx="110" cy="52" r="3" />
      <!-- 바퀴 -->
      <rect class="alm-ph-wheel" x="84" y="82" width="6" height="14" rx="3" />
      <rect class="alm-ph-wheel" x="130" y="82" width="6" height="14" rx="3" />
    </g>

    <!-- 물음표 말풍선 -->
    <g class="alm-ph-bubble">
      <rect x="138" y="46" width="26" height="22" rx="8" />
      <text x="151" y="62" text-anchor="middle">?</text>
    </g>
  </svg>
  <strong class="alm-ph-title">${title}</strong>
  <small class="alm-ph-detail">${detail}</small>
  <code class="alm-ph-path">maps/${mapName || '—'}/${is3d ? 'cloud.pcd' : 'grid.pgm'}</code>`;
}

/**
 * 컨테이너 위에 placeholder 를 띄우거나 지운다.
 *
 * @param {Element} container  position:relative 인 스테이지 요소
 * @param {boolean} show
 * @param {'cloud'|'grid'} kind
 * @param {string} mapName
 * @returns {boolean} 실제로 표시 중인지
 */
export function toggleMapPlaceholder(container, show, kind, mapName) {
  if (!container) return false;
  let node = container.querySelector(':scope > .alm-placeholder');

  if (!show) {
    if (node) node.remove();
    return false;
  }

  if (!node) {
    node = document.createElementNS('http://www.w3.org/1999/xhtml', 'div');
    node.className = 'alm-placeholder';
    container.appendChild(node);
  }
  const signature = `${kind}|${mapName}`;
  if (node.dataset.signature !== signature) {
    node.dataset.signature = signature;
    node.innerHTML = markup(kind, mapName);
  }
  return true;
}

/** SVG 안(navigation 지도)에 넣기 위한 foreignObject 판. */
export function toggleSvgMapPlaceholder(svg, show, mapName) {
  if (!svg) return false;
  const existing = svg.querySelector('#almMapPlaceholder');
  if (!show) {
    if (existing) existing.remove();
    return false;
  }
  if (existing) return true;

  const box = document.createElementNS(SR, 'foreignObject');
  box.setAttribute('id', 'almMapPlaceholder');
  box.setAttribute('x', '0');
  box.setAttribute('y', '0');
  box.setAttribute('width', '900');
  box.setAttribute('height', '620');
  const holder = document.createElementNS('http://www.w3.org/1999/xhtml', 'div');
  holder.setAttribute('class', 'alm-placeholder in-svg');
  holder.innerHTML = markup('grid', mapName);
  box.appendChild(holder);
  svg.appendChild(box);
  return true;
}
