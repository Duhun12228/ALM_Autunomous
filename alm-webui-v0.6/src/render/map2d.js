/**
 * 2D 운용 지도 — OccupancyGrid + 로봇 자세를 실데이터로 그린다.
 *
 * 목업의 #staticMapLayer 는 좌표가 박힌 SVG 도형이고, 로봇 마커도 고정
 * transform 이었다. 여기서는 /map 을 받아 이미지로 굽고, 지도 좌표계(m)와
 * SVG 뷰박스(900×620) 사이의 변환을 한 번 계산해 둔 뒤 그 변환으로 로봇을 놓는다.
 *
 * 계산한 변환은 window.ALM_MAP_TRANSFORM 으로 내보낸다 — app.js 의
 * pixelToMap() 이 이걸 보고 마우스 좌표를 실제 미터로 바꾼다.
 */
import { quaternionToYaw } from '../bridge/decoders.js';

const VIEW_W = 900;
const VIEW_H = 620;

// nav_msgs/OccupancyGrid 의 data 는 -1(미관측) 또는 0~100(점유 확률)
const UNKNOWN = -1;

// 로봇 마커 배율 [SVG단위/m]. robotScale() 주석 참조.
//   기본값 : /map 이 오기 전에 쓰는 자리표시값 (index.html 의 scale 과 같은 값)
//   하한   : 차체 1.37 m 가 최소 14 SVG단위로 보이도록 (1.37 x 10 ≈ 14)
const DEFAULT_ROBOT_PX_PER_M = 50;
const MIN_ROBOT_PX_PER_M = 10;

export class Map2D {
  constructor() {
    this.svg = document.querySelector('#navigationMap');
    this.staticLayer = document.querySelector('#staticMapLayer');
    this.robotLayer = document.querySelector('#robotLayer');
    this.scanLayer = document.querySelector('#scanLayer');
    this.transform = null;
    this.image = null;
  }

  /** OccupancyGrid → SVG <image>. 맵은 거의 안 바뀌므로 받을 때마다 다시 굽는다. */
  onOccupancyGrid(msg) {
    const { width, height, resolution } = msg.info;
    if (!width || !height) return;

    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    const image = ctx.createImageData(width, height);
    const data = msg.data;

    for (let i = 0; i < width * height; i += 1) {
      // OccupancyGrid 의 원점은 좌하단, 캔버스는 좌상단 — 행을 뒤집는다
      const row = height - 1 - Math.floor(i / width);
      const target = (row * width + (i % width)) * 4;
      const value = data[i];

      let r; let g; let b; let a;
      if (value === UNKNOWN) {
        [r, g, b, a] = [11, 14, 19, 255];        // --void 에 가까운 미관측
      } else if (value >= 65) {
        [r, g, b, a] = [140, 152, 168, 255];     // 점유(벽)
      } else {
        [r, g, b, a] = [27, 34, 43, 255];        // 자유공간
      }
      image.data[target] = r;
      image.data[target + 1] = g;
      image.data[target + 2] = b;
      image.data[target + 3] = a;
    }
    ctx.putImageData(image, 0, 0);

    // 지도 전체가 뷰박스에 들어가도록 등비 축소하고 가운데 정렬
    const scale = Math.min(VIEW_W / width, VIEW_H / height);
    const drawW = width * scale;
    const drawH = height * scale;
    const padX = (VIEW_W - drawW) / 2;
    const padY = (VIEW_H - drawH) / 2;

    if (!this.image) {
      this.staticLayer.replaceChildren();
      this.image = document.createElementNS('http://www.w3.org/2000/svg', 'image');
      this.image.setAttribute('image-rendering', 'pixelated');
      this.staticLayer.appendChild(this.image);
    }
    this.image.setAttribute('x', padX);
    this.image.setAttribute('y', padY);
    this.image.setAttribute('width', drawW);
    this.image.setAttribute('height', drawH);
    this.image.setAttribute('href', canvas.toDataURL('image/png'));

    // 지도 좌표(m) → SVG 픽셀. y 는 화면에서 아래로 증가하므로 부호가 뒤집힌다.
    //   svgX = originX_px + x_m * pxPerMeter
    //   svgY = originY_px - y_m * pxPerMeter
    const pxPerMeter = scale / resolution;
    this.transform = {
      scale: pxPerMeter,
      offsetX: padX - msg.info.origin.position.x * pxPerMeter,
      offsetY: padY + drawH + msg.info.origin.position.y * pxPerMeter,
      resolution,
    };
    window.ALM_MAP_TRANSFORM = this.transform;
  }

  /** 지도 좌표(m) → SVG 좌표. 변환이 아직 없으면 null. */
  toSvg(x, y) {
    if (!this.transform) return null;
    return {
      x: this.transform.offsetX + x * this.transform.scale,
      y: this.transform.offsetY - y * this.transform.scale,
    };
  }

  /**
   * 로봇 마커의 화면 배율 [SVG단위/m].
   *
   * 마커는 index.html 에 **미터로** 그려져 있으므로(footprint 실측값), 여기서
   * px/m 를 그대로 곱하면 화면상 크기가 실제 차체와 일치한다. 예전에는 마커가
   * 픽셀로 그려져 있어서, 맵이 바뀌어 축척이 달라지면 차가 실제보다 2배 넘게
   * 커 보였다 — 좁은 통로를 지날 수 있는지 눈으로 가늠할 수 없었다.
   *
   * 하한을 두는 이유: 아주 넓은 맵에서는 px/m 가 작아져 차가 몇 픽셀로 뭉개진다.
   * 그때는 크기의 정확성보다 '어디 있는지 보이는 것'이 우선이다.
   */
  robotScale() {
    if (!this.transform) return DEFAULT_ROBOT_PX_PER_M;
    return Math.max(MIN_ROBOT_PX_PER_M, this.transform.scale);
  }

  /** /Odometry 또는 TF 로 얻은 자세를 로봇 마커에 반영. */
  setRobotPose(pose) {
    if (!pose || !this.robotLayer) return;
    const point = this.toSvg(pose.x, pose.y);
    if (!point) return;
    // SVG rotate 는 시계방향, ROS yaw 는 반시계방향
    const degrees = -pose.yaw * 180 / Math.PI;
    const scale = this.robotScale();
    this.robotLayer.setAttribute('transform',
      `translate(${point.x.toFixed(2)} ${point.y.toFixed(2)}) `
      + `rotate(${degrees.toFixed(1)}) scale(${scale.toFixed(3)})`);
  }

  onOdometry(msg) {
    const pose = msg.pose?.pose;
    if (!pose) return;
    this.setRobotPose({
      x: pose.position.x,
      y: pose.position.y,
      yaw: quaternionToYaw(pose.orientation),
    });
  }

  /**
   * /scan (LaserScan) → 지도 위의 점. 로봇 자세 기준으로 각 레이를 놓는다.
   * 점이 수천 개라 SVG 요소를 매번 만들면 느리다 — 하나의 path 로 합친다.
   */
  onLaserScan(msg, pose) {
    if (!this.scanLayer || !this.transform || !pose) return;
    const parts = [];
    const step = Math.max(1, Math.floor(msg.ranges.length / 720));

    for (let i = 0; i < msg.ranges.length; i += step) {
      const range = msg.ranges[i];
      if (!Number.isFinite(range) || range < msg.range_min || range > msg.range_max) continue;
      const angle = msg.angle_min + i * msg.angle_increment + pose.yaw;
      const point = this.toSvg(
        pose.x + range * Math.cos(angle),
        pose.y + range * Math.sin(angle));
      if (point) parts.push(`M${point.x.toFixed(1)} ${point.y.toFixed(1)}h1`);
    }
    this.scanLayer.replaceChildren();
    if (!parts.length) return;

    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', parts.join(''));
    path.setAttribute('stroke', '#6FA8FF');
    path.setAttribute('stroke-width', '2');
    path.setAttribute('stroke-linecap', 'round');
    path.setAttribute('opacity', '.75');
    this.scanLayer.appendChild(path);
  }

  /** nav_msgs/Path → 경로 레이어. globalPathLayer / localPathLayer 공용. */
  onPath(msg, layerSelector, color, width) {
    const layer = document.querySelector(layerSelector);
    if (!layer || !this.transform) return;
    layer.replaceChildren();
    if (!msg.poses?.length) return;

    const points = msg.poses
      .map((entry) => this.toSvg(entry.pose.position.x, entry.pose.position.y))
      .filter(Boolean)
      .map((point, index) => `${index ? 'L' : 'M'}${point.x.toFixed(1)} ${point.y.toFixed(1)}`);
    if (!points.length) return;

    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', points.join(''));
    path.setAttribute('fill', 'none');
    path.setAttribute('stroke', color);
    path.setAttribute('stroke-width', String(width));
    path.setAttribute('stroke-linecap', 'round');
    layer.appendChild(path);
  }
}
