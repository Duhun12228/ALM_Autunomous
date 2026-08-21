# Jetson ↔ STM32 UART 연동 — 변경사항 및 확인 절차

> 작성: 2026-08-05 / 브랜치 `dev/fastlio2-sc`
> 목적: 자율주행 연산 결과가 UART로 STM32까지 내려가는지 제어단과 함께 확인하기 위한 정리.
> 프로토콜 상세 규격은 `ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md` (v2) 참고.

---

## 0. 30초 요약

- STM32 펌웨어(`jetson_uart_parse.m` + `FourWIS_DrivingAlgorithm`) 실물을 열어보니
  **기존 워크스페이스 규격과 전혀 맞지 않았다.** 그래서 Jetson 송신부를 STM32 규격에 맞췄다.
- 핵심: **역기구학은 STM32가 이미 한다.** Jetson은 RC 조종기처럼
  **조향각 1개 + 속도 1개 + 모드**만 보낸다. (twist 18바이트 → **10바이트**)
- 새 파일 2개(`fourwis_encode.py`, `uart_teleop.py`), 수정 6개. 빌드/검증 완료.
- 확인은 **ROS 없이 `uart_teleop.py` 단독 실행**으로 가능하다. 제어단과 마주앉아 바로 쓸 수 있다.
- 미팅에서 받아와야 할 것: **`CONS` 11개 값 + 휠 반경 + 감속비**, 그리고 크랩/제로턴 **회전 방향 부호**.

---

## 1. 왜 바꿨나 (배경)

기존 `docs/uart_protocol.md` v1은 **우리가 임시로 정해둔 방향**이었고, 실제 펌웨어와 3가지가 어긋났다.

| | 기존 v1 (우리 가정) | 실제 STM32 |
|---|---|---|
| payload | 18B `vx, vy, wz, mode, flags, seq` | **10B** `steer_deg, speed_rpm, mode, flags` |
| 역기구학 | STM32가 twist를 받아서 계산 | **이미 STM32 안에 구현됨** (RC 조종기 입력을 기대) |
| 하드웨어 | 2축 조향 + 4구동 | **4축 독립 조향** + 2축(2채널) 구동 드라이버 |
| mode 값 | 0=normal, 1=crab, 2=spin, 3=auto | **0=정지, 1=일반, 3=크랩, 4=제로턴** |

`length` 값이 다르면 STM32 파서가 프레임을 **전부 거절**한다(`buf(4) ~= PAYLOAD_LEN` → 1바이트씩 버림).
즉 기존 코드 그대로 연결했으면 **통신이 0% 성립**했다.

`mode` 불일치는 특히 위험했다. STM32는 `mode=0`을 **정지**로 쓰는데 우리 문서는 `0=normal`이었다.
그대로 붙였으면 **통신 끊김 시 정지가 아니라 주행 명령**이 됐을 것이다.

---

## 2. 프로토콜 v2 (확정)

### 프레임 (16 바이트)

```
+------+------+----------+--------+-------------+-----------+
| 0xAA | 0x55 |   0x01   |  0x0A  | payload(10) | crc16 (2) |
+------+------+----------+--------+-------------+-----------+
  sync0  sync1  msg_type   length                 big-endian
```

- CRC16-CCITT (poly `0x1021`, init `0xFFFF`), 범위 = `[msg_type, length, payload]`
- payload 내부 수치는 little-endian, CRC만 big-endian

### Payload (10 바이트)

| off | type | 필드 | 의미 |
|-----|------|------|------|
| 0 | float32 | `steer_deg` | 조향 명령 [deg]. **음수 = 좌회전** |
| 4 | float32 | `speed_rpm` | 구동 명령 [rpm]. 일반 주행은 **내측 전륜** 기준 |
| 8 | uint8 | `mode` | 아래 표 |
| 9 | uint8 | `flags` | bit0=enable, bit1=estop (STM32 미사용, 추후 과제) |

Python: `struct.pack('<ffBB', steer_deg, speed_rpm, mode, flags)`
C: `#pragma pack(1) struct { float steer_deg, speed_rpm; uint8_t mode, flags; }`

### mode 값 — 펌웨어 실측

| mode | 동작 | 비고 |
|------|------|------|
| **0** | **정지** | 파서 timeout(200ms) 기본값 |
| **1** | 일반 주행 | Ackermann + 후륜 보조조향(RWS) |
| 2 | 자가추종 | anchor 입력 없어 **현재 정지 처리** |
| **3** | 크랩 | 조향각 `CONS(8)` 고정, 부호로 방향만 |
| **4** | 제로턴 | 조향각 `CONS(9)` 고정, 부호로 방향만 |
| 5·그외 | 정지 | |

### ⚠ 반드시 기억할 3가지

1. **부호 반전** — 일반 주행은 STM32 내부에서 `normalDriveMode(-steer_deg, ...)`로 호출된다.
   → **좌회전이면 음수를 보내야 한다.** (RC 조종기 좌우반전 보정이 코드에 박혀 있음)
2. **`speed_rpm`은 차체 속도가 아니다** — 좌회전 시 `rpm_FL`, 우회전 시 `rpm_FR`에 그대로 들어가고
   나머지 3륜은 STM32가 비율 스케일한다. 직진일 때만 4륜 동일.
3. **크랩/제로턴은 연속 제어가 안 된다** — `steer_deg`의 **부호만** 쓰고 크기는 무시된다.
   각도가 STM32 상수로 고정이라 임의 방향 크랩이 불가능하다. 속도 크기만 조절 가능.

### 물리 계층 (`.ioc` 확인 완료)

| 항목 | 값 |
|---|---|
| STM32 | STM32F407VET6 |
| 포트 | **USART1** — PA9(TX) / PA10(RX), **115200** 8N1 |
| DMA | RX = DMA2_Stream2 **Circular**, TX = DMA2_Stream7 |
| Jetson | `/dev/ttyTHS1` |
| 배선 | 3.3V TTL 직결, **TX↔RX 교차**, **GND 공통 필수** |

---

## 3. 추가/수정된 파일 전체 목록

### 새로 만든 것 (2개)

| 파일 | 줄수 | 역할 |
|---|---|---|
| `alm_base_control/scripts/fourwis_encode.py` | 360 | **twist → `steer_deg`/`speed_rpm`/`mode` 변환 + 프레임 생성.** ROS 의존성 없음 |
| ~~`alm_base_control/scripts/uart_teleop.py`~~ | — | *(후속 커밋에서 제거)* 포트 이중 점유 방지를 위해 ROS 텔레옵(`keyboard_teleop.py`) + `cmd_arbiter` 로 대체. → [control_arbitration.md](control_arbitration.md) |

**`fourwis_encode.py`** 안에 든 것:
- `FourWISParams` — 기하 상수 (STM32 `CONS`와 맞춰야 하는 값들)
- `encode(vx, vy, wz, drive_mode, stopped, params)` → `(steer_deg, speed_rpm, mode_id, note)`
- `build_command_frame()` / `crc16_ccitt()` — `mcu_bridge`와 동일한 바이트 생성
- `min_turn_radius()` / `max_angular_speed()` — 기구학 한계 계산
- `python3 fourwis_encode.py` 로 **단독 자체검증** 실행 가능

### 수정한 것 (6개)

| 파일 | 변경 내용 |
|---|---|
| `alm_msgs/msg/McuCommand.msg` | `steer_deg`, `speed_rpm`, `mode_id` 3필드 추가. `cmd_vel`/`drive_mode`는 디버그용으로 유지 |
| `alm_mcu_interface/scripts/mcu_bridge.py` | `CMD_FMT`: `<fffBBI>`(18B) → **`<ffBB>`(10B)**. `MODE_TO_ID` 삭제(잘못된 매핑). `on_command()`가 새 필드를 패킹 |
| `alm_base_control/scripts/command_manager.py` | `fourwis_encode` 호출 추가. 4WIS 파라미터 12개 선언. **기구학 한계 자가진단 로그** 추가 |
| `alm_base_control/config/base_control.yaml` | 4WIS 파라미터 12개(`##CONFIRM##`) 추가. `odom_topic`을 `/odometry/filtered` → **`/Odometry`** 로 수정 |
| `alm_base_control/CMakeLists.txt` | 새 스크립트 2개 설치 등록 |
| `alm_mcu_interface/docs/uart_protocol.md` | **v2로 전면 개정** — mode 표를 펌웨어 실측값으로 교체, 부호 규약·제약 명시 |

### `odom_topic` 수정은 별건 (안전)

주행 모드에선 EKF를 끄므로 `/odometry/filtered`가 **존재하지 않는다**.
그래서 `have_odom`이 영원히 `False`가 되어 **오도메트리 워치독이 조용히 비활성**돼 있었다.
→ 측위가 죽어도 로봇이 마지막 명령으로 계속 갈 수 있는 상태였다.
FAST-LIO 출력(`/Odometry`)을 보도록 바꿔서 "측위 끊기면 0.5초 내 정지"가 살아났다.

---

## 4. 데이터 흐름

```
Nav2 ──/cmd_vel──▶ command_manager ──/mcu/command──▶ mcu_bridge ──UART──▶ STM32
                   ├ drive_mode 해석 (auto→normal/spin/crab)   ├ 10B payload
                   ├ 속도/가속 제한, e-stop, 워치독              ├ CRC16-CCITT
                   └ fourwis_encode: twist→steer/rpm/mode       └ pyserial write
                                                                        │
                                              jetson_uart_parse.m ◀─────┘
                                                    │ (싱크→길이→CRC→언팩)
                                                    ▼
                                        FourWIS_DrivingAlgorithm
                                          → 4륜 조향각(count) + 4륜 rpm
                                                    │
                                                  CAN 500kbps
                                                    ▼
                                          조향·인휠 드라이버
```

**중요: 선만 연결한다고 자동으로 나가지 않는다.** `command_manager`와 `mcu_bridge`
**두 노드가 떠 있어야** 바이트가 나간다.

---

## 5. 확인 절차 — 터미널에서 뭘 켜야 하나

### 사전 준비 (Jetson, 1회)

```bash
# 시리얼 권한
sudo usermod -aG dialout $USER      # 후 재로그인

# ⚠ Jetson THS1이 시리얼 콘솔에 물려 있으면 포트가 안 열린다
sudo systemctl disable --now nvgetty

# 확인
ls -l /dev/ttyTHS1
```

빌드:
```bash
source /opt/ros/humble/setup.bash
cd ~/ALM_fastlio2-sc/ALM_auto_ws          # ← worktree 경로 주의 (README는 ALM_Autunomous 기준)
colcon build --packages-select alm_msgs alm_base_control alm_mcu_interface
source install/setup.bash
```

---

### 방법 A. 키보드 텔레옵 (ROS 경유)

> **변경됨:** 포트를 직접 열던 `uart_teleop.py` 는 제거되었다. 포트는 `mcu_bridge`
> 하나만 소유한다(이중 점유 시 프레임 깨짐). 수동 조작도 ROS 를 거쳐 동작권 중재기
> (`cmd_arbiter`)를 통과한다. 배선·상태기계·서비스 사용법은
> **[docs/control_arbitration.md](control_arbitration.md)** 참고.

```bash
ros2 run alm_base_control keyboard_teleop.py
#   t 동작권잡기, w/s 전후, a/d 회전, q/e 게걸음, space 비상정지, c 해제,
#   1/3/4/0 모드, r 반납(auto), x 종료
```

> ⚠ **바퀴가 실제로 돕니다. 잭업(차량 들어올림) 상태에서 먼저 확인할 것.**
> 먼저 `t` 로 동작권을 잡아야 명령이 반영된다. 텔레옵이 명령을 끊으면 정지 유지(HELD).

**Jetson 쪽 단독 배선 점검 (STM32 없이):**
`/dev/ttyTHS1`의 **TX와 RX를 서로 단락**시키고 위 도구를 실행하면,
자기가 보낸 프레임이 `RX ...` 로 화면에 표시된다. 포트·권한·배선이 살아있다는 뜻.

---

### 방법 B. 자율주행 전체 체인 (ROS)

**최소 구성 — 터미널 4개** (LiDAR 없이 UART 체인만 보고 싶을 때)

```bash
# 터미널 1 — UART 브리지
ros2 run alm_mcu_interface mcu_bridge.py --ros-args \
  -p port:=/dev/ttyTHS1 -p baudrate:=115200

# 터미널 2 — 명령 관리자 (twist → 4WIS 변환)
ros2 run alm_base_control command_manager.py --ros-args \
  --params-file ~/ALM_fastlio2-sc/ALM_auto_ws/install/alm_base_control/share/alm_base_control/config/base_control.yaml

# 터미널 3 — 명령 주입 (Nav2 대신 수동)
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.15}}"

# 터미널 4 — 실제로 뭘 보내는지 확인 ★
ros2 topic echo /mcu/command
```

터미널 4에서 이렇게 나오면 정상이다:
```yaml
cmd_vel: {linear: {x: 0.25}, angular: {z: 0.15}}
drive_mode: normal
steer_deg: -30.0          # ← 좌회전이므로 음수
speed_rpm: 25.354772567749023
mode_id: 1                # ← 일반 주행
```

**전체 스택 — 터미널 1개** (LiDAR·EKF까지 포함, 실주행용)
```bash
ros2 launch alm_bringup robot.launch.py
# 여기에 navigation.launch.py 를 얹으면 Nav2까지
```

---

## 6. 제어단과 함께 볼 체크리스트

| # | 확인 항목 | Jetson 쪽 | STM32 쪽에서 봐야 할 것 |
|---|---|---|---|
| 1 | 배선·포트 | TX/RX 단락 루프백에서 RX 표시됨 | — |
| 2 | 프레임 수신 | `--dry-run` hex와 대조 | 파서 `valid` 플래그가 `true`로 유지 |
| 3 | 값 일치 | `--mode direct`로 "20 300 1" | `steer_deg=20, speed=300, mode=1` 복원 |
| 4 | timeout 동작 | 도구를 Ctrl-C로 종료 | 200ms 후 `mode=0`, `valid=false` → 정지 |
| 5 | **좌우 방향** | `--mode sequence` 좌회전 단계 | 바퀴가 **실제로 좌회전**하는지 |
| 6 | **크랩 방향** | sequence 크랩좌/우 | 의도한 방향인지 → 다르면 `crab_steer_sign` 뒤집기 |
| 7 | **제로턴 방향** | sequence 제로턴좌/우 | 의도한 방향인지 → 다르면 `spin_steer_sign` 뒤집기 |
| 8 | 속도 스케일 | rpm 값 확인 | 실제 바퀴 회전수와 맞는지 → 안 맞으면 `wheel_radius_m`/`gear_ratio` |
| 9 | 전체 체인 | `ros2 topic echo /mcu/command` | 같은 값이 STM32에 도착하는지 |

**5~7번이 이번 미팅의 핵심 산출물이다.** 부호 3개를 확정하면 `base_control.yaml`에 바로 반영한다.

STM32 쪽 관측은 Simulink **Data Inspector / XCP**로 `valid`, `steer_deg`, `speed`, `mode`를
플롯하면 가장 확실하다. (파서 출력에 `valid`를 넣어둔 이유)

---

## 7. 미팅에서 받아와야 할 값

`FourWIS_DrivingAlgorithm`의 `CONS` 벡터가 `.slx` 안에 없고 외부 m파일/워크스페이스 변수다.
이 값들이 있어야 Jetson의 변환식이 실제 궤적과 일치한다.

```
CONS(1)  RWS_percent           ← 후륜 보조조향 비율 (%)     ★ 중요
CONS(2)  B  [mm]               ← 휠베이스                   ★ 중요
CONS(3)  T  [mm]               ← 전체 윤거                  ★ 중요
CONS(4)  straight_angle [deg]  ← 조향 데드밴드
CONS(5)  steer_count_per_rev   ← 조향 엔코더 카운트/회전
CONS(6)  max_steering_rpm
CONS(7)  Ts_control            ← 제어 주기 (50Hz면 0.02)
CONS(8)  crab_angle_deg        ← 크랩 고정 각도
CONS(9)  zero_turn_angle_deg   ← 제로턴 고정 각도
CONS(10) crab_rpm_scale
CONS(11) zero_turn_rpm_scale
```

추가로:
- **`wheel_radius`** [m] — rpm 환산에 직접 들어감 ★
- **감속비** — CAN PID 207의 rpm이 모터축인지 휠축인지 ★
- 조향 각도 기구 한계 [deg]
- 구동 드라이버 최대 rpm
- Jetson을 USART1(PA9/PA10)에 물리는 게 맞는지 (USART2/3 용도 확인)

받으면 `base_control.yaml`의 `##CONFIRM##` 12개를 채운다.

---

## 8. 알려진 제약 / 아직 안 된 것

### 🔴 `max_angular_z`가 기구학 한계를 초과한다

현재 잠정값 기준 자가진단 로그:
```
4WIS: 최소 회전반경 2.08 m, vx=0.45 m/s 에서 가능한 최대 wz=0.22 rad/s
경고: max_angular_z(0.8)가 기구학 한계(0.22)를 초과합니다.
```
`vx=0.25, wz=0.15`(R=1.67m)만 줘도 조향이 **-30°로 포화**된다.
이대로 Nav2를 붙이면 계획 경로와 실제 궤적이 계속 어긋난다.
→ CONS 실측값 받은 뒤 Nav2 각속도 제한을 다시 잡아야 한다. (`rws`가 크면 최소반경이 줄어든다)

### 크랩/제로턴은 이산 동작

조향각이 STM32 상수 고정이라 Nav2가 임의 방향 크랩이나 연속 wz를 요구해도 못 낸다.
`auto` 모드에서 spin 전환 시 회전 속도는 rpm으로만 조절된다.

### State 업링크(STM32→Jetson) 없음

- **자율주행에는 지장 없다.** Nav2의 odom은 FAST-LIO `/Odometry`가 담당하고
  엔코더 오도메트리는 경로에 없다. `mcu_bridge`는 수신이 없으면 조용히 송신만 한다.
- 못 쓰는 것: `/wheel_odom`, `/joint_states`(RViz 바퀴 시각화), MCU fault 피드백
- 규격은 `uart_protocol.md`에 예정안으로 남겨뒀다.

### e-stop 미연동

`flags`(bit0=enable, bit1=estop)를 프레임에 채워 보내지만 **STM32 파서가 언팩하지 않는다**
(`last_mode = buf(13)`까지만 읽고 `buf(14)`는 무시). 현재는 RC 컨트롤러 최상위 오버라이드로
대체하기로 했고, e-stop 연동은 추후 과제.

---

## 9. 검증 기록 (이미 완료한 것)

| 단계 | 방법 | 결과 |
|---|---|---|
| 역기구학 | `python3 fourwis_encode.py` — 역산→정방향 재계산으로 twist 복원 | 통과 |
| 프레임 왕복 | `jetson_uart_parse.m`을 Python으로 1:1 포팅해 대조. 청크 분할·앞잡음·CRC 오염·timeout(11틱→정지) | 통과 |
| 프레임 일치 | `build_command_frame`이 문서 예시와 **바이트 단위 일치** (셀프테스트로 고정) | 통과 |
| E2E | 가상 시리얼(pty)에 실제 `mcu_bridge`+`command_manager` 물리고 `/cmd_vel` 발행 | 통과 |
| 단독 도구 | `uart_teleop --mode sequence` 실제 포트 송신 → STM32 파서로 복원 | 205 프레임, 정렬오류 0 |
| 빌드 | `colcon build` (alm_msgs, alm_base_control, alm_mcu_interface) | 통과 |

E2E 결과:
```
명령 전 -> mode=0 (정지)
명령 후 -> steer=-30.000  rpm=25.35  mode=1  valid=True    ← 좌회전이 음수 ✓
```

검증용 예시 프레임 (`steer=12.5, rpm=300, mode=1, flags=1`):
```
AA 55 01 0A 00 00 48 41 00 00 96 43 01 01 8E DD
```

---

## 10. 다음 할 일

1. 미팅에서 `CONS` 11개 + 휠반경 + 감속비 확보 → `base_control.yaml` `##CONFIRM##` 채우기
2. 잭업 상태에서 `--mode sequence` 돌려 **크랩/제로턴 부호 3개 확정**
3. 확정된 최소 회전반경에 맞춰 **Nav2 각속도 제한 재조정**
4. (추후) State 업링크 규격 협의, e-stop `flags` 연동
