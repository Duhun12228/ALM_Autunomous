/**
 * foxglove_bridge 클라이언트 — 구독 전용.
 *
 * 이 파일에는 publish 경로가 없다. 의도적이다. Phase 1~2 는 읽기 전용이고,
 * 브리지 쪽 allowlist(alm_bringup/config/foxglove_webui.yaml)도 클라이언트
 * publish 를 막아 두었다. 명령은 Phase 3 에서 alm_web_backend 를 통해 나간다.
 *
 * 거동:
 *   - 원하는 토픽을 미리 등록해 두면(subscribe), 브리지가 그 토픽을 광고하는
 *     시점에 자동으로 붙는다. 노드가 나중에 떠도 알아서 연결된다.
 *   - 연결이 끊기면 지수 백오프로 재접속하고, 재접속 후 구독을 복구한다.
 *   - 토픽별 수신 시각을 기록해 '살아있는지' 판정에 쓴다(HUD 점등).
 */
import { FoxgloveClient } from '@foxglove/ws-protocol';

import { makeDecoder } from './decoders.js';

const RECONNECT_MIN_MS = 500;
const RECONNECT_MAX_MS = 8000;

/**
 * 제시할 서브프로토콜. 서버가 아는 것을 골라 준다.
 *
 * foxglove_bridge 3.x(Rust SDK)는 'foxglove.sdk.v1' 만 받는데,
 * @foxglove/ws-protocol 0.8 의 SUPPORTED_SUBPROTOCOL 은 아직
 * 'foxglove.websocket.v1' 이다. 클라이언트 상수만 믿고 연결하면
 * **101 이 아닌 응답으로 핸드셰이크가 거부**되고, 브리지 로그에는
 * "handshake failed" 만 남아 원인을 찾기 어렵다. 둘 다 제시해 두면
 * 구·신 브리지 양쪽에 붙는다. 프레이밍은 동일해 디코딩은 그대로 통한다.
 */
export const SUBPROTOCOLS = ['foxglove.sdk.v1', FoxgloveClient.SUPPORTED_SUBPROTOCOL];

export class RosBridge {
  /**
   * @param {string} url 예: ws://10.14.145.64:8765
   */
  constructor(url) {
    this.url = url;
    this.client = null;
    this.status = 'disconnected';   // disconnected | connecting | connected

    this.handlers = new Map();      // topic -> Set<fn(message, meta)>
    this.decoders = new Map();      // channelId -> fn(data)
    this.channelsByTopic = new Map();
    this.subscriptions = new Map(); // subscriptionId -> topic
    this.lastSeen = new Map();      // topic -> performance.now()

    this.statusListeners = new Set();
    this.reconnectDelay = RECONNECT_MIN_MS;
    this.reconnectTimer = null;
    this.closedByUser = false;
  }

  /** 상태 변화 구독. ('connected' | 'connecting' | 'disconnected') */
  onStatus(listener) {
    this.statusListeners.add(listener);
    listener(this.status);
    return () => this.statusListeners.delete(listener);
  }

  _setStatus(status) {
    if (this.status === status) return;
    this.status = status;
    this.statusListeners.forEach((listener) => listener(status));
  }

  /**
   * 토픽 구독 등록. 연결 전에 불러도 되고, 같은 토픽에 여러 핸들러도 된다.
   * @returns {() => void} 해제 함수
   */
  subscribe(topic, handler) {
    if (!this.handlers.has(topic)) this.handlers.set(topic, new Set());
    this.handlers.get(topic).add(handler);

    // 이미 연결돼 있고 그 토픽이 광고된 상태라면 즉시 붙는다
    if (this.status === 'connected' && this.channelsByTopic.has(topic)) {
      this._attach(topic);
    }
    return () => {
      const set = this.handlers.get(topic);
      if (set) set.delete(handler);
    };
  }

  /** 토픽의 마지막 수신 이후 경과 [ms]. 한 번도 못 받았으면 Infinity. */
  ageOf(topic) {
    const seen = this.lastSeen.get(topic);
    return seen === undefined ? Infinity : performance.now() - seen;
  }

  /** 토픽이 maxAgeMs 안에 갱신되고 있는가. HUD 점등 판정용. */
  isFresh(topic, maxAgeMs = 3000) {
    return this.ageOf(topic) <= maxAgeMs;
  }

  connect() {
    this.closedByUser = false;
    this._open();
  }

  close() {
    this.closedByUser = true;
    clearTimeout(this.reconnectTimer);
    this.client?.close();
    this.client = null;
    this._setStatus('disconnected');
  }

  _open() {
    clearTimeout(this.reconnectTimer);
    this._setStatus('connecting');

    let socket;
    try {
      socket = new WebSocket(this.url, SUBPROTOCOLS);
    } catch (error) {
      console.warn('[bridge] WebSocket 생성 실패', error);
      this._scheduleReconnect();
      return;
    }
    socket.binaryType = 'arraybuffer';

    const client = new FoxgloveClient({ ws: socket });
    this.client = client;

    client.on('open', () => {
      this.reconnectDelay = RECONNECT_MIN_MS;
      this._setStatus('connected');
    });

    client.on('error', (error) => {
      // 재연결은 close 에서 일괄 처리한다. 여기서는 기록만.
      console.warn('[bridge] 오류', error);
    });

    client.on('close', () => {
      this.decoders.clear();
      this.channelsByTopic.clear();
      this.subscriptions.clear();
      this.client = null;
      this._setStatus('disconnected');
      if (!this.closedByUser) this._scheduleReconnect();
    });

    client.on('advertise', (channels) => {
      for (const channel of channels) {
        this.channelsByTopic.set(channel.topic, channel);
        if (this.handlers.has(channel.topic)) this._attach(channel.topic);
      }
    });

    client.on('unadvertise', (channelIds) => {
      for (const id of channelIds) this.decoders.delete(id);
    });

    client.on('message', ({ subscriptionId, timestamp, data }) => {
      const topic = this.subscriptions.get(subscriptionId);
      if (topic === undefined) return;

      this.lastSeen.set(topic, performance.now());
      const decode = this.decoders.get(subscriptionId);
      if (!decode) return;

      let message;
      try {
        message = decode(data);
      } catch (error) {
        console.warn(`[bridge] ${topic} 디코딩 실패`, error);
        return;
      }
      // 한 핸들러가 던져도 나머지는 계속 받아야 한다
      for (const handler of this.handlers.get(topic) ?? []) {
        try {
          handler(message, { topic, timestamp });
        } catch (error) {
          console.error(`[bridge] ${topic} 핸들러 오류`, error);
        }
      }
    });
  }

  _attach(topic) {
    const channel = this.channelsByTopic.get(topic);
    if (!channel || !this.client) return;
    // 이미 붙어 있으면 중복 구독하지 않는다
    for (const existing of this.subscriptions.values()) {
      if (existing === topic) return;
    }

    const decode = makeDecoder(channel);
    if (!decode) return;

    const subscriptionId = this.client.subscribe(channel.id);
    this.subscriptions.set(subscriptionId, topic);
    this.decoders.set(subscriptionId, decode);
  }

  _scheduleReconnect() {
    clearTimeout(this.reconnectTimer);
    const delay = this.reconnectDelay;
    this.reconnectDelay = Math.min(delay * 2, RECONNECT_MAX_MS);
    this.reconnectTimer = setTimeout(() => this._open(), delay);
  }
}

/**
 * 브리지 주소 결정.
 *   1) ?bridge=ws://host:port  (현장에서 빠르게 갈아끼우기)
 *   2) localStorage 'alm-bridge-url'
 *   3) 페이지를 서빙한 호스트의 8765 포트 — 젯슨에서 서빙하면 그대로 맞는다
 */
export function resolveBridgeUrl() {
  const fromQuery = new URLSearchParams(location.search).get('bridge');
  if (fromQuery) return fromQuery;

  const stored = localStorage.getItem('alm-bridge-url');
  if (stored) return stored;

  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = location.hostname || 'localhost';
  return `${protocol}//${host}:8765`;
}
