# ALM Jetson ↔ STM32 UART 프로토콜 규격 v2

Jetson(ROS 2, `mcu_bridge`) 과 STM32 펌웨어 사이의 UART 통신 규격입니다.
역기구학(바퀴별 조향각/속도)은 **STM32의 `FourWIS_DrivingAlgorithm` 이 담당**하고,
Jetson 은 RC 조종기와 같은 규격(조향각 1개 + 속도 1개 + 모드)만 보냅니다.

> **v1 → v2 변경**: Command payload 가 `vx/vy/wz` twist 18바이트에서
> `steer_deg/speed_rpm/mode/flags` **10바이트**로 바뀌었습니다. STM32 측
> `jetson_uart_parse.m` 파서 규격에 맞춘 것이며, mode 값 정의도 실제 펌웨어
> 코드 기준으로 전면 교체됐습니다(v1 문서의 mode 표는 폐기).

## 물리 계층
- 인터페이스: UART (Jetson `/dev/ttyTHS1` ↔ STM32 **USART1** PA9/TX, PA10/RX)
- Baud: **115200** (8N1) — `.ioc` 확인 완료 (USART1.BaudRate=115200)
- STM32 RX 는 DMA2_Stream2 **Circular** 모드 → 스트림이 끊기지 않음
- 바이트 순서: payload 내부 수치는 **little-endian**, CRC 는 **big-endian**
- 배선: 3.3V TTL 직결, TX↔RX 교차, **GND 공통 필수**

## 프레임 구조 (양방향 공통)

```
+------+------+----------+--------+------------------+-----------+
| 0xAA | 0x55 | msg_type | length |   payload[len]   | crc16 (2) |
+------+------+----------+--------+------------------+-----------+
  sync0  sync1   1 byte    1 byte     length bytes     big-endian
```

- `msg_type`: `0x01` = Command(Jetson→STM32), `0x02` = State(STM32→Jetson)
- `length`: payload 바이트 수 (**Command=10**, State=63)
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

## Command payload — `msg_type = 0x01` (Jetson → STM32), 10 bytes

| off | type    | 필드         | 단위/의미 |
|-----|---------|--------------|-----------|
| 0   | float32 | `steer_deg`  | 조향 명령 [deg]. **음수 = 좌회전** (아래 부호 규약 참고) |
| 4   | float32 | `speed_rpm`  | 구동 명령 [rpm]. 일반 주행은 **내측 전륜** 기준 |
| 8   | uint8   | `mode`       | 주행 모드 (아래 표) |
| 9   | uint8   | `flags`      | bit0=enable_motors, bit1=emergency_stop |

- C 구조체 (`#pragma pack(1)`): `struct { float steer_deg, speed_rpm; uint8_t mode, flags; }`
- Python: `struct.pack('<ffBB', steer_deg, speed_rpm, mode, flags)`
- **명령 timeout**: STM32 파서는 10틱(50Hz 기준 200ms) 이상 유효 프레임이 없으면
  `steer=0, speed=0, mode=0(정지)` 로 자동 복귀합니다.

### mode 값 (STM32 `FourWIS_DrivingAlgorithm` 실측 기준)

| mode | 동작 | 비고 |
|------|------|------|
| **0** | **정지** | timeout 기본값 |
| **1** | 일반 주행 | Ackermann + 후륜 보조조향(RWS) |
| 2 | 자가추종 | anchor 입력이 없어 **현재 정지 처리됨** |
| **3** | 크랩 | 조향각은 `CONS(8)` **고정**, 부호로 방향만 결정 |
| **4** | 제로턴 | 조향각은 `CONS(9)` **고정**, 부호로 방향만 결정 |
| 5·그외 | 정지 | |

### ⚠ 부호 규약 (실수하기 쉬운 부분)

일반 주행은 STM32 내부에서 `normalDriveMode(-steer_deg, ...)` 로 **부호가 반전되어**
호출됩니다(RC 조종기 좌우 반전 보정). 따라서

- 좌회전(`wz > 0`) → **음수** `steer_deg` 전송
- 우회전(`wz < 0`) → **양수** `steer_deg` 전송

### `speed_rpm` 의 의미

일반 주행에서 STM32 는 좌회전 시 `rpm_FL = speed_rpm`, 우회전 시 `rpm_FR = speed_rpm`
으로 두고 나머지 3륜을 여기서 비율 스케일합니다. 즉 **차체 중심속도가 아니라
내측 전륜의 rpm** 입니다. 직진일 때만 4륜이 동일합니다.

### 크랩/제로턴의 제약

두 모드는 `steer_deg` 의 **부호만** 사용하고 크기는 무시합니다. 조향각이 STM32
상수로 고정돼 있어 **임의 방향의 연속 제어가 불가능**하며, Jetson 이 조절할 수
있는 것은 속도 크기(`speed_rpm`)와 방향(부호) 뿐입니다.

### 역기구학은 STM32 담당

바퀴별 조향각/속도는 `FourWIS_DrivingAlgorithm` 이 계산합니다
(**4륜 독립 조향** `angle_FL/FR/RL/RR`, 4륜 rpm, 조향 출력 단위는 **엔코더 count**).
Jetson 측 twist → `steer_deg/speed_rpm` 역산은
`alm_base_control/scripts/fourwis_encode.py` 에 있으며, 그 안의 기하 상수
(`wheelbase_m`, `track_m`, `rws_ratio`)는 STM32 `CONS(2)/CONS(3)/CONS(1)` 과
**반드시 같은 값**이어야 명령과 실제 궤적이 일치합니다.

## State payload — `msg_type = 0x02` (STM32 → Jetson), 63 bytes

> **현재 미구현 (예정 규격).** STM32 측에 업링크 송신부가 아직 없습니다.
> 없어도 자율주행은 성립합니다 — Nav2 의 odom 은 FAST-LIO `/Odometry` 가
> 담당하고 엔코더 오도메트리는 경로에 없기 때문입니다. `mcu_bridge` 는
> State 프레임이 오지 않으면 조용히 다운링크만 수행합니다.
> 다만 `/wheel_odom`·`/joint_states`·MCU fault 피드백은 발행되지 않습니다.

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
- Jetson 프레임 송신: `alm_mcu_interface/scripts/mcu_bridge.py` (`CMD_FMT`)
- Jetson twist 변환: `alm_base_control/scripts/fourwis_encode.py`
  (단독 실행하면 자체 검증: `python3 fourwis_encode.py`)
- **포트 단일 소유 원칙**: 시리얼 포트(`/dev/ttyTHS1`)는 **`mcu_bridge` 하나만** 연다.
  두 프로세스가 동시에 쓰면 프레임 바이트가 섞여 STM32 파서가 동기를 잃는다.
- **수동 조작(텔레옵)**: `alm_base_control/scripts/keyboard_teleop.py` (ROS 노드).
  포트를 직접 열지 않고 `/cmd_vel_teleop` 를 발행 → `cmd_arbiter` 가 동작권을 주면
  자율과 같은 경로(cmd_arbiter → command_manager → mcu_bridge)로 내려간다.
  자세한 배선/사용법은 `docs/control_arbitration.md` 참고.
  ```bash
  ros2 run alm_base_control keyboard_teleop.py   # t=동작권 잡기, WASD 조작, r=반납
  ```
- 변환 파라미터: `alm_base_control/config/base_control.yaml` 의 `##CONFIRM##` 항목
- 메시지 정의: `alm_msgs/msg/McuCommand.msg`, `alm_msgs/msg/McuState.msg`
- STM32 파서: `jetson_uart_parse.m`, 주행 알고리즘: `FourWIS_DrivingAlgorithm`
- 레이아웃 변경 시 이 문서 · `mcu_bridge.py`(`CMD_FMT`) · `jetson_uart_parse.m`
  (`PAYLOAD_LEN` 과 언팩 오프셋)을 **함께** 수정할 것.

### 검증용 예시 프레임
`steer_deg=12.5, speed_rpm=300, mode=1(일반), flags=0x01` →
```
AA 55 01 0A 00 00 48 41 00 00 96 43 01 01 8E DD
```
STM32 파서에 그대로 넣으면 위 값이 복원되어야 합니다.
