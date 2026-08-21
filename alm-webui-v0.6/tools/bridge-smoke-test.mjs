/**
 * 브리지 ↔ 디코더 스모크 테스트 (브라우저 없이).
 *
 *   node tools/bridge-smoke-test.mjs [ws://호스트:8765]
 *
 * 화면 코드에서 가장 검증이 어려운 구간 — foxglove_bridge 연결, 채널 광고,
 * CDR 디코딩 — 을 브라우저를 띄우지 않고 확인한다. src/bridge/decoders.js 를
 * 그대로 import 하므로, 여기서 통과하면 브라우저에서도 같은 결과가 나온다.
 *
 * 마지막에 클라이언트 publish 를 시도해 **거부되는지**까지 확인한다.
 * 이건 통과가 아니라 '실패해야 정상'인 항목이다.
 */
// @foxglove/ws-protocol 은 CommonJS 라 Node 의 ESM 로더가 named export 를
// 찾지 못한다. Vite 는 번들링하며 알아서 풀어주므로 브라우저 코드(src/)에서는
// named import 를 그대로 쓴다 — 이 우회는 Node 로 직접 실행하는 여기서만 필요하다.
import wsProtocol from '@foxglove/ws-protocol';

import { makeDecoder } from '../src/bridge/decoders.js';

const { FoxgloveClient } = wsProtocol;

// src/bridge/ros-bridge.js 와 같은 목록이어야 의미가 있다.
// (그쪽은 @foxglove/ws-protocol 을 named import 해서 Node 로 직접 못 읽으므로
//  여기서는 값만 맞춰 둔다 — 한쪽을 바꾸면 다른 쪽도 바꿀 것)
const SUBPROTOCOLS = ['foxglove.sdk.v1', FoxgloveClient.SUPPORTED_SUBPROTOCOL];

const url = process.argv[2] ?? 'ws://localhost:8765';
const DURATION_MS = 12000;

const WANTED = [
  '/alm/jetson_stats',
  '/alm/map_inventory',
  '/mcu/state',
  '/cmd_arbiter/owner',
  '/drive_mode/effective',
  '/map',
  '/livox/lidar',
  // 활성 맵에 저장된 점군. latched 라 상시 있어야 한다.
  '/alm/prior_cloud',
];

/**
 * 매핑/측위 스택이 돌 때만 존재하는 토픽. 없다고 실패로 세면 SLAM 이 꺼져 있는
 * 평상시마다 빨간 불이 켜져서, 정작 진짜 고장이 났을 때 아무도 안 본다.
 * 있으면 내용을 확인하고, 없으면 '해당 없음' 으로 넘어간다.
 */
const OPTIONAL = new Set([
  '/Odometry',            // FAST-LIO
  '/cloud_registered',    // FAST-LIO
]);
WANTED.push(...OPTIONAL);

const seen = new Map();       // topic -> { count, sample }
const subscriptions = new Map();
const decoders = new Map();

const socket = new WebSocket(url, SUBPROTOCOLS);
socket.binaryType = 'arraybuffer';
const client = new FoxgloveClient({ ws: socket });

let advertised = [];
let serverCapabilities = null;

client.on('serverInfo', (info) => {
  serverCapabilities = info.capabilities ?? [];
});

client.on('open', () => console.log(`연결됨: ${url}`));
client.on('error', (error) => console.error('오류:', error.message ?? error));

client.on('advertise', (channels) => {
  for (const channel of channels) {
    advertised.push(channel.topic);
    if (!WANTED.includes(channel.topic)) continue;

    const decode = makeDecoder(channel);
    if (!decode) {
      console.error(`  ✗ ${channel.topic}: 디코더 생성 실패 (${channel.schemaName})`);
      continue;
    }
    const id = client.subscribe(channel.id);
    subscriptions.set(id, channel.topic);
    decoders.set(id, decode);
  }
});

client.on('message', ({ subscriptionId, data }) => {
  const topic = subscriptions.get(subscriptionId);
  if (!topic) return;
  const entry = seen.get(topic) ?? { count: 0, sample: null };
  entry.count += 1;
  if (!entry.sample) {
    try {
      entry.sample = decoders.get(subscriptionId)(data);
    } catch (error) {
      entry.sample = { __decodeError: String(error) };
    }
  }
  seen.set(topic, entry);
});

setTimeout(async () => {
  console.log(`\n광고된 채널 ${advertised.length}개`);
  console.log(`서버 capabilities: ${JSON.stringify(serverCapabilities)}`);

  console.log('\n── 구독 결과 ──────────────────────────────');
  let failures = 0;
  for (const topic of WANTED) {
    const entry = seen.get(topic);
    if (!entry) {
      if (OPTIONAL.has(topic)) {
        console.log(`  · ${topic.padEnd(24)} 해당 없음 (매핑/측위 미기동)`);
        continue;
      }
      console.log(`  ✗ ${topic.padEnd(24)} 수신 없음`);
      failures += 1;
      continue;
    }
    if (entry.sample?.__decodeError) {
      console.log(`  ✗ ${topic.padEnd(24)} 디코딩 실패: ${entry.sample.__decodeError}`);
      failures += 1;
      continue;
    }
    const hz = (entry.count / (DURATION_MS / 1000)).toFixed(1);
    console.log(`  ✓ ${topic.padEnd(24)} ${String(entry.count).padStart(4)}건 (~${hz} Hz)  ${summarize(topic, entry.sample)}`);
  }

  console.log('\n── 쓰기 차단 확인 (실패해야 정상) ─────────');
  // 연결 자체가 안 됐으면 capabilities 는 null 이다. 그걸 '차단됨'으로 읽으면
  // 브리지가 죽어 있을 때 보안 검사가 통과로 나온다 — 가장 위험한 오판이다.
  if (serverCapabilities === null) {
    console.log('  ! 서버 정보를 받지 못해 판정 불가 (연결 실패)');
    console.log('\n결과: 실패 (연결 안 됨)');
    process.exit(1);
  }
  const canPublish = serverCapabilities.includes('clientPublish');
  if (canPublish) {
    console.log('  ✗ 서버가 clientPublish 를 광고함 — 브라우저가 토픽을 발행할 수 있습니다!');
    failures += 1;
  } else {
    console.log('  ✓ clientPublish 미광고 — 브라우저에 발행 경로가 없습니다');
  }
  const canCallService = (serverCapabilities ?? []).includes('services');
  if (canCallService) {
    console.log('  ✗ 서버가 services 를 광고함 — 브라우저가 서비스를 호출할 수 있습니다!');
    failures += 1;
  } else {
    console.log('  ✓ services 미광고 — 서비스 호출 경로가 없습니다');
  }

  console.log(failures === 0 ? '\n결과: 통과' : `\n결과: 실패 ${failures}건`);
  client.close();
  process.exit(failures === 0 ? 0 : 1);
}, DURATION_MS);

function summarize(topic, msg) {
  switch (topic) {
    case '/alm/jetson_stats':
      return `cpu ${msg.cpu_percent?.toFixed(1)}% gpu ${msg.gpu_percent?.toFixed(1)}% `
        + `ram ${msg.ram_used_gb?.toFixed(2)}/${msg.ram_total_gb?.toFixed(2)}GB `
        + `tj ${msg.temp_tj?.toFixed(1)}°C ${msg.power_w?.toFixed(1)}W ${msg.net_interface}`;
    case '/alm/map_inventory': {
      const active = (msg.maps ?? []).find((m) => m.active);
      const assets = (active?.assets ?? [])
        .map((a) => `${a.kind}${a.present ? (a.stale ? '⚠' : '✓') : '✗'}`).join(' ');
      return `root=${msg.root?.split('/').slice(-3).join('/')} `
        + `maps=${msg.maps?.length} active=${msg.active_map} `
        + `complete=${active?.complete} [${assets}]`;
    }
    case '/mcu/state':
      return `batt ${msg.battery_voltage?.toFixed(1)}V steer[${msg.steer_angle?.length}] `
        + `wheels[${msg.wheel_speed?.length}] estop=${msg.emergency_stop} fault=${msg.fault}`;
    case '/cmd_arbiter/owner':
    case '/drive_mode/effective':
      return `"${msg.data}"`;
    case '/Odometry':
      return `x ${msg.pose?.pose?.position?.x?.toFixed(2)} y ${msg.pose?.pose?.position?.y?.toFixed(2)}`;
    case '/map':
      return `${msg.info?.width}×${msg.info?.height} @${msg.info?.resolution?.toFixed(3)}m `
        + `origin(${msg.info?.origin?.position?.x?.toFixed(2)}, ${msg.info?.origin?.position?.y?.toFixed(2)}) `
        + `data ${msg.data?.length}`;
    case '/livox/lidar':
    case '/cloud_registered':
    case '/alm/prior_cloud':
      return `${msg.width}×${msg.height} step ${msg.point_step} `
        + `fields[${msg.fields?.map((f) => f.name).join(',')}] frame=${msg.header?.frame_id}`;
    default:
      return '';
  }
}
