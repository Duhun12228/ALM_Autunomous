/**
 * 브리지 구독 장부 — 채널/구독/디코더의 대응 관계 한 곳.
 *
 * 왜 따로 뺐나: 이 장부가 RosBridge 안의 세 인라인 핸들러(advertise /
 * unadvertise / _attach)에 흩어져 있었고, 그래서 **키를 잘못 쓴 게 눈에 안
 * 띄었다.** unadvertise 가 `decoders.delete(channelId)` 를 했는데 decoders 의
 * 키는 subscriptionId 였다. 아무것도 안 지워졌고 subscriptions 에 죽은 토픽이
 * 남았으며, 그 유령 때문에 재광고 시 '이미 구독 중'으로 판정돼 재구독을
 * 건너뛰었다. SLAM 을 껐다 켜면 /cloud_registered 가 영영 안 들어왔다.
 *
 * 외부 의존이 없다 — Node 로 그대로 import 해서 테스트할 수 있다.
 * (@foxglove/ws-protocol 은 CJS 라 ros-bridge.js 는 Node 로 못 읽는다)
 */
export class SubscriptionRegistry {
  constructor() {
    this.channelsByTopic = new Map();   // topic -> channel
    this.topicByChannelId = new Map();  // channelId -> topic
    this.subIdByTopic = new Map();      // topic -> subscriptionId
    this.topicBySubId = new Map();      // subscriptionId -> topic
    this.decoderBySubId = new Map();    // subscriptionId -> fn(data)
  }

  /** advertise 수신. 같은 토픽이 새 채널로 다시 오면 채널만 갈아끼운다. */
  setChannel(channel) {
    const previous = this.channelsByTopic.get(channel.topic);
    if (previous && previous.id !== channel.id) {
      this.topicByChannelId.delete(previous.id);
    }
    this.channelsByTopic.set(channel.topic, channel);
    this.topicByChannelId.set(channel.id, channel.topic);
  }

  channelFor(topic) {
    return this.channelsByTopic.get(topic);
  }

  hasChannel(topic) {
    return this.channelsByTopic.has(topic);
  }

  /** 구독이 성립했을 때 기록. */
  addSubscription(topic, subscriptionId, decode) {
    this.subIdByTopic.set(topic, subscriptionId);
    this.topicBySubId.set(subscriptionId, topic);
    this.decoderBySubId.set(subscriptionId, decode);
  }

  isSubscribed(topic) {
    return this.subIdByTopic.has(topic);
  }

  topicFor(subscriptionId) {
    return this.topicBySubId.get(subscriptionId);
  }

  decoderFor(subscriptionId) {
    return this.decoderBySubId.get(subscriptionId);
  }

  /**
   * unadvertise 수신. 그 채널에 딸린 **모든** 상태를 지운다.
   * @returns {string|undefined} 사라진 토픽 이름
   */
  removeChannel(channelId) {
    const topic = this.topicByChannelId.get(channelId);
    if (topic === undefined) return undefined;
    this.topicByChannelId.delete(channelId);
    this.channelsByTopic.delete(topic);
    const subscriptionId = this.subIdByTopic.get(topic);
    if (subscriptionId !== undefined) {
      this.subIdByTopic.delete(topic);
      this.topicBySubId.delete(subscriptionId);
      this.decoderBySubId.delete(subscriptionId);
    }
    return topic;
  }

  clear() {
    this.channelsByTopic.clear();
    this.topicByChannelId.clear();
    this.subIdByTopic.clear();
    this.topicBySubId.clear();
    this.decoderBySubId.clear();
  }
}
