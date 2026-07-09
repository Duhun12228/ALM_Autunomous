# ALM Jetson ↔ STM32 UART 프로토콜 규격 v1

Jetson(ROS 2, `mcu_bridge`) 과 STM32 펌웨어 사이의 UART 통신 규격입니다.
STM32 팀은 이 문서 그대로 파싱/생성하면 됩니다. (기구학은 STM32가 담당)

## 물리 계층
- 인터페이스: UART (Jetson `/dev/ttyTHS1`)
- Baud: **115200** (8N1) — 양쪽 동일하게 맞출 것
- 바이트 순서: payload 내부 수치는 **little-endian**, CRC 는 **big-endian**

## 프레임 구조 (양방향 공통)

```
+------+------+----------+--------+------------------+-----------+
| 0xAA | 0x55 | msg_type | length |   payload[len]   | crc16 (2) |
+------+------+----------+--------+------------------+-----------+
  sync0  sync1   1 byte    1 byte     length bytes     big-endian
```

- `msg_type`: `0x01` = Command(Jetson→STM32), `0x02` = State(STM32→Jetson)
- `length`: payload 바이트 수 (Command=18, State=63)
- `crc16`: **CRC16-CCITT** (다항식 `0x1021`, 초기값 `0xFFFF`),
  계산 범위 = `[msg_type, length, payload...]` (sync 2바이트 제외), 전송은 big-endian(상위바이트 먼저)

### CRC16-CCITT 참조 구현 (C)
```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

## Command payload — `msg_type = 0x01` (Jetson → STM32), 18 bytes

| off | type    | 필드          | 단위/의미 |
|-----|---------|---------------|-----------|
| 0   | float32 | `vx`          | 전진 선속도 [m/s] (base_link +x) |
| 4   | float32 | `vy`          | 측면 선속도 [m/s] (base_link +y, crab용) |
| 8   | float32 | `wz`          | 요 각속도 [rad/s] (CCW +) |
| 12  | uint8   | `drive_mode`  | 0=normal, 1=crab, 2=spin, 3=auto |
| 13  | uint8   | `flags`       | bit0=enable_motors, bit1=emergency_stop |
| 14  | uint32  | `sequence`    | 명령 시퀀스 (증가) |

- C 구조체 (`#pragma pack(1)`): `struct { float vx, vy, wz; uint8_t mode, flags; uint32_t seq; }`
- Jetson 은 이미 `auto`를 normal/spin/crab 으로 해석해서 보내므로 STM32는 주로 0~2 만 받게 됩니다.
- `enable_motors=0` 또는 `emergency_stop=1` 이면 STM32는 즉시 정지해야 합니다.
- **명령 timeout**: STM32는 최근 Command 를 200 ms(권장) 이상 못 받으면 정지할 것.

### STM32 역기구학 (여기서 구현)
2축 조향 + 4구동 기준. 조향 드라이버 앞축/뒤축 각 1개, 구동 드라이버 4개.
```
# 지오메트리 (CAD 실측, base_link=차체중심 기준, [m])
front_x = +0.6106,  rear_x = -0.3010,  half_track = 0.500,  wheel_radius = 0.103

normal : 각 바퀴 속도벡터 v_i = (vx - wz*y_i,  vy + wz*x_i)
         앞축 조향각 = atan2(vy + wz*front_x, vx),  뒤축 = atan2(vy + wz*rear_x, vx)
         바퀴 각속도 = |v_i| / wheel_radius   (전/후진 부호 주의)
spin   : 제자리 회전. 앞축/뒤축을 서로 반대로 꺾어 순수 wz 생성 (vx=vy=0)
crab   : 앞축=뒤축 동일 각도로 병진 (wz=0). 조향각 = atan2(vy, vx)
```
> 정확한 각도 배분/부호/Ackermann 보정은 STM32 팀 기구학 상수에 맞춰 구현.

## State payload — `msg_type = 0x02` (STM32 → Jetson), 63 bytes

| off | type    | 필드            | 단위/의미 |
|-----|---------|-----------------|-----------|
| 0   | uint32  | `sequence`      | 상태 시퀀스 |
| 4   | float32 | `odom_x`        | 정기구학 적분 위치 x [m] (odom frame) |
| 8   | float32 | `odom_y`        | 위치 y [m] |
| 12  | float32 | `odom_theta`    | 헤딩 [rad] |
| 16  | float32 | `vx`            | 측정 전진 선속도 [m/s] |
| 20  | float32 | `vy`            | 측정 측면 선속도 [m/s] |
| 24  | float32 | `wz`            | 측정 요 각속도 [rad/s] |
| 28  | float32 | `steer_front`   | 앞축 조향각 [rad] |
| 32  | float32 | `steer_rear`    | 뒤축 조향각 [rad] |
| 36  | float32 | `wheel_fl`      | 앞좌 구동 각속도 [rad/s] |
| 40  | float32 | `wheel_fr`      | 앞우 구동 각속도 [rad/s] |
| 44  | float32 | `wheel_rl`      | 뒤좌 구동 각속도 [rad/s] |
| 48  | float32 | `wheel_rr`      | 뒤우 구동 각속도 [rad/s] |
| 52  | float32 | `battery_voltage` | [V] |
| 56  | float32 | `battery_current` | [A] |
| 60  | uint8   | `status_flags`  | bit0=motors_enabled, bit1=estop, bit2=command_timeout, bit3=fault |
| 61  | uint16  | `fault_code`    | 결함 코드 (0=정상) |

- C 구조체 (`#pragma pack(1)`): `struct { uint32_t seq; float odom_x,odom_y,odom_theta, vx,vy,wz, steer_f,steer_r, w_fl,w_fr,w_rl,w_rr, batt_v,batt_c; uint8_t flags; uint16_t fault; }`
- 권장 발행 주기: **50~100 Hz**

### STM32 정기구학 (여기서 구현: 엔코더 → 오도메트리)
```
# 입력: 4구동 엔코더 각속도 w_i [rad/s], 2조향 엔코더 각도 steer_f/steer_r [rad]
# 각 바퀴 접지속도 s_i = w_i * wheel_radius, 방향 = 해당 축 조향각
# 최소자승/평균으로 body twist (vx, vy, wz) 추정 후 dt 적분 → (odom_x, odom_y, odom_theta)
```

## 참고
- Jetson 측 구현: `alm_mcu_interface/scripts/mcu_bridge.py`
- 메시지 정의: `alm_msgs/msg/McuCommand.msg`, `alm_msgs/msg/McuState.msg`
- 레이아웃 변경 시 이 문서 · `mcu_bridge.py`(`CMD_FMT`/`STATE_FMT`) · STM32 구조체를 함께 수정.
