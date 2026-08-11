/**
 * 회귀 테스트 — 토픽이 사라졌다 다시 나타나면 재구독되는가.
 *
 * 왜 필요한가: SLAM 을 껐다 켜면 /cloud_registered 가 unadvertise → advertise
 * 된다. 예전 RosBridge 는 unadvertise 때 `decoders.delete(channelId)` 만 했는데
 * decoders 의 키는 subscriptionId 였다. 아무것도 안 지워졌고 subscriptions 에
 * 죽은 토픽이 남았으며, 그 유령 때문에 재광고 시 '이미 구독 중'으로 판정돼
 * 재구독을 건너뛰었다. **브라우저는 그 뒤로 점군을 영영 못 받았고, 매핑이
 * 진행돼도 화면에 아무것도 안 그려졌다.**
 *
 * ros-bridge.js 는 @foxglove/ws-protocol(CJS)을 named import 해서 Node 로 직접
 * 못 읽는다. 그래서 장부 로직만 subscription-registry.js 로 떼어 두고 여기서
 * 그 실물을 검증한다.
 *
 *   node tools/bridge-resubscribe-test.mjs
 */
import { SubscriptionRegistry } from '../src/bridge/subscription-registry.js';

let failures = 0;
const check = (name, got, want) => {
  const ok = got === want;
  if (!ok) failures += 1;
  console.log(`  ${ok ? '✓' : '✗'} ${name}${ok ? '' : `  (기대 ${want}, 실제 ${got})`}`);
};

const channel = (topic, id) => ({ id, topic });
const registry = new SubscriptionRegistry();

// RosBridge._attach() 와 같은 순서로 장부를 쓴다
let nextSubId = 100;
const attach = (topic) => {
  if (!registry.hasChannel(topic)) return false;
  if (registry.isSubscribed(topic)) return false;
  registry.addSubscription(topic, (nextSubId += 1), () => ({}));
  return true;
};

console.log('\n── SLAM 시작 (최초 광고) ──');
registry.setChannel(channel('/cloud_registered', 17));
check('구독 성립', attach('/cloud_registered'), true);
check('메시지 라우팅 가능', registry.topicFor(101), '/cloud_registered');
check('디코더 등록됨', typeof registry.decoderFor(101), 'function');

console.log('── SLAM 종료 (unadvertise) ──');
check('사라진 토픽 반환', registry.removeChannel(17), '/cloud_registered');
check('구독 해제됨', registry.isSubscribed('/cloud_registered'), false);
check('채널 해제됨', registry.hasChannel('/cloud_registered'), false);
check('디코더 정리됨', registry.decoderFor(101), undefined);
check('라우팅 정리됨', registry.topicFor(101), undefined);

console.log('── SLAM 재시작 (새 채널 번호로 재광고) ──');
registry.setChannel(channel('/cloud_registered', 18));
check('재구독됨 (이게 핵심)', attach('/cloud_registered'), true);
check('새 subscriptionId 로 라우팅', registry.topicFor(102), '/cloud_registered');

console.log('── 중복 광고 방지 ──');
registry.setChannel(channel('/cloud_registered', 18));
check('중복 구독 안 함', attach('/cloud_registered'), false);

console.log('── 같은 토픽이 채널 번호만 바뀌어 재광고 ──');
registry.setChannel(channel('/cloud_registered', 19));
check('옛 채널 id 는 정리됨', registry.removeChannel(18), undefined);
check('새 채널 id 로 해제 가능', registry.removeChannel(19), '/cloud_registered');

console.log('── 모르는 채널 해제 ──');
check('undefined 반환 (예외 없음)', registry.removeChannel(999), undefined);

console.log(failures === 0 ? '\n결과: 통과' : `\n결과: 실패 ${failures}건`);
process.exit(failures === 0 ? 0 : 1);
