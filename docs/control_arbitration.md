# 제어 경로와 동작권 중재 (cmd_arbiter)

## 원칙: 시리얼 포트는 mcu_bridge 하나만 소유

STM32 로 내려가는 UART(`/dev/ttyTHS1`)는 **`mcu_bridge` 단 하나**만 연다.
두 프로세스가 같은 포트에 프레임을 쓰면 바이트가 섞여(interleave) STM32 파서가
프레임 동기를 잃는다 — 재현이 어렵고 위험한 통신 깨짐. 그래서 자율이든 수동이든
모든 명령은 ROS 토픽으로 모아 한 경로로만 내려보낸다.

## 배선

```
Nav2(DWB) ──/cmd_vel───────┐
   (수동)  ──/drive_mode────┤
                            ▼
keyboard_teleop ─/cmd_vel_teleop──▶ [cmd_arbiter] ─/cmd_vel_mux───▶ command_manager ─/mcu/command─▶ mcu_bridge ─UART─▶ STM32
       │        ─/drive_mode_teleop─▶     │        ─/drive_mode_mux─▶                                 (유일한 포트 소유자)
       │        ─/emergency_stop(Bool)────┼─────────────────────────▶ (command_manager 직접 구독)
       └─ 서비스 /cmd_arbiter/set_owner ──┘        └─/cmd_arbiter/owner (latched, 관측용)
```

- `cmd_arbiter` 는 자율 소스와 텔레옵 소스 중 **동작권을 가진 쪽**만 `_mux` 토픽으로 통과시킨다.
- 4WIS 변환(twist → steer/rpm/mode)과 안전 게이팅은 **command_manager 한 곳**에서만 한다.
  텔레옵은 twist 만 쏜다.
- `/emergency_stop` 은 arbiter 를 거치지 않고 command_manager 가 직접 처리하므로,
  어느 소유자든 즉시 하드 정지된다.

## 동작권 상태기계

| 상태 | 진입 | 출력 |
|------|------|------|
| `auto` (기본) | 부팅 시, 또는 `set_owner("auto")` | Nav 소스(`/cmd_vel`, `/drive_mode`) 통과 |
| `teleop` | `set_owner("teleop")` | 텔레옵 소스 통과. 텔레옵 메시지 도착시각을 하트비트로 추적 |
| `teleop(held)` | `teleop` 중 텔레옵 명령이 `teleop_timeout_sec`(기본 0.5s) 끊김 | **0 twist 유지**. 자율로 자동 복귀하지 않음 |

- **인계 안전**: 텔레옵이 동작권을 쥔 채 명령이 끊기면(키 입력 정지·프로세스 사망 포함)
  즉시 정지하고 그대로 유지한다. 자율은 **`set_owner("auto")` 를 명시적으로 호출해야만** 재개된다.
- 각 소스는 freshness 게이팅: 원본이 `*_timeout_sec` 넘게 끊기면 0 twist 를 낸다
  (arbiter 가 50Hz 로 재발행하므로 command_manager 의 자체 timeout 이 대신 못 잡아준다).

## 사용법

### 키보드 텔레옵
```bash
ros2 run alm_base_control keyboard_teleop.py
```
| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| `t` | 동작권 잡기(teleop) | `r` | 동작권 반납(auto) |
| `w`/`s` | 전진/후진 | `a`/`d` | 좌/우 회전 |
| `q`/`e` | 좌/우 게걸음(crab) | `z` | 속도 0 리셋 |
| `space` | 비상정지 ON | `c` | 비상정지 해제 |
| `1`/`3`/`4`/`0` | drive_mode = normal/crab/spin/auto | `x`/Ctrl-C | 종료 |

먼저 `t` 로 동작권을 잡아야 명령이 반영된다. ⚠ 바퀴가 실제로 돈다 — 잭업 후 확인.

### 서비스로 직접 전환
```bash
ros2 service call /cmd_arbiter/set_owner alm_msgs/srv/SetControlOwner "{owner: teleop}"
ros2 service call /cmd_arbiter/set_owner alm_msgs/srv/SetControlOwner "{owner: auto}"
ros2 topic echo /cmd_arbiter/owner        # 현재 소유자 관측 (auto | teleop | teleop(held))
```

### 런타임 파라미터 (즉시 반영)
```bash
ros2 param set /cmd_arbiter teleop_timeout_sec 1.0
ros2 param set /cmd_arbiter default_owner teleop
ros2 param set /cmd_arbiter publish_rate_hz 30.0
```

## 관련 파일
- `alm_base_control/scripts/cmd_arbiter.py` — 중재 노드
- `alm_base_control/scripts/keyboard_teleop.py` — ROS 키보드 텔레옵
- `alm_base_control/config/base_control.yaml` — `cmd_arbiter` / `command_manager` 파라미터
- `alm_base_control/launch/base_control.launch.py` — 두 노드 기동
- `alm_msgs/srv/SetControlOwner.srv` — 동작권 전환 서비스
- `alm_mcu_interface/scripts/mcu_bridge.py` — 유일한 시리얼 소유자(문지기)
