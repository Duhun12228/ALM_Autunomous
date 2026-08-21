/**
 * 채널 스키마 → 디코더.
 *
 * foxglove_bridge 는 채널마다 (encoding, schemaName, schemaEncoding, schema) 를
 * 광고한다. ROS 2 에서는 encoding="cdr", schemaEncoding="ros2msg"(연결된 .msg
 * 정의문) 또는 "ros2idl" 이다. 그 정의문을 파싱해 MessageReader 를 만들어 두면
 * 이후 프레임은 ArrayBuffer → 객체로 바로 풀린다.
 *
 * 채널당 한 번만 만들고 캐시한다 — 파싱은 비싸고 스키마는 안 바뀐다.
 */
import { parse as parseRos2msg } from '@foxglove/rosmsg';
import { MessageReader } from '@foxglove/rosmsg2-serialization';

const textDecoder = new TextDecoder();

/** 채널 하나에 대한 디코딩 함수를 만든다. 못 만들면 null. */
export function makeDecoder(channel) {
  if (channel.encoding !== 'cdr') {
    console.warn(`[bridge] ${channel.topic}: 지원하지 않는 encoding=${channel.encoding}`);
    return null;
  }

  // foxglove_bridge(ROS 2)는 ros2msg — 연결된 .msg 정의문 — 로 광고한다.
  // ros2idl 은 별도 파서(@foxglove/omgidl-parser)가 필요해 여기서는 지원하지
  // 않는다. 조용히 빈 값을 그리느니 무엇을 못 읽었는지 남기고 넘어간다.
  if (channel.schemaEncoding && channel.schemaEncoding !== 'ros2msg') {
    console.warn(
      `[bridge] ${channel.topic}: 지원하지 않는 schemaEncoding=${channel.schemaEncoding}`);
    return null;
  }

  const schemaText = typeof channel.schema === 'string'
    ? channel.schema
    : textDecoder.decode(channel.schema);

  let definitions;
  try {
    definitions = parseRos2msg(schemaText, { ros2: true });
  } catch (error) {
    console.warn(`[bridge] ${channel.topic}: 스키마 파싱 실패`, error);
    return null;
  }

  const reader = new MessageReader(definitions);
  return (data) => reader.readMessage(data);
}

/** ROS Time({sec,nanosec}) → 밀리초. 화면 표시용. */
export function rosTimeToMs(time) {
  if (!time) return 0;
  return time.sec * 1000 + time.nanosec / 1e6;
}

/** geometry_msgs/Quaternion → yaw [rad]. 2D 화면은 yaw 만 쓴다. */
export function quaternionToYaw(q) {
  if (!q) return 0;
  const { x = 0, y = 0, z = 0, w = 1 } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}
