# 제어 루프 현황 분석 — 경로계획에서 바퀴까지

> **이 문서는** `feat/nav2-hybrid-astar` 브랜치의 자율주행 제어 사슬 전체를 코드 기준으로
> 추적하고, **어디가 폐루프(피드백 있음)이고 어디가 개루프(피드백 없음)인지**를 정리한
> 것입니다. 이 프로젝트를 처음 보는 사람도 읽을 수 있게 전제부터 씁니다.
>
> 작성 기준: 커밋 `caa3567` (Nav2 경로 파이프라인: Hybrid-A* + ConstrainedSmoother + MPPI)
> 검증 방법: 코드 정독 + `fourwis_encode.py` 실행 + `command_manager` 틱 루프 재현 시뮬레이션

---

## 0. 5분 요약

경로계획(Hybrid-A* → ConstrainedSmoother → MPPI)은 잘 설계돼 있고 숫자도 전부 기구
상수에서 유도돼 있습니다. **문제는 계획이 아니라 "명령이 실제로 실행됐는지 확인하는
경로"가 없다는 것**입니다.

| # | 문제 | 심각도 | 확인 방법 |
|---|---|---|---|
| 1 | `/Odometry.twist` 가 항상 0 → MPPI 속도 피드백 사망 | 🔴 높음 | 코드 정독 (확정) |
| 2 | 조향 명령 대비 실제 조향각을 **아무도 안 봄** | 🔴 높음 | 코드 정독 (확정) |
| 3 | 출발 시 조향 명령이 20 ms 만에 0° → 30° (1500 deg/s) | 🟠 중간 | 시뮬레이션 (확정) |
| 4 | 동적 장애물 반응이 최악 4.3 초 / 1.94 m | 🟠 중간 | 지연 예산 계산 |
| 5 | 구동계 상수 다수가 `##CONFIRM##` 미확정 | 🟡 낮음 | 코드 주석 |

**한 줄로**: 계획은 정교한데 실행 결과를 되먹임하는 선이 끊겨 있습니다.

---

## 1. 이 로봇이 어떤 로봇인가 (전제 지식)

### 1.1 4륜 독립조향(4WIS) 플랫폼

바퀴 4개가 각각 조향됩니다. 하지만 **STM32 펌웨어가 3가지 모드로만 동작**합니다:

| 모드 | STM32 `mode` | 동작 | 제자리 회전 |
|---|---|---|---|
| `normal` | 1 | 자동차형. 앞바퀴 조향 + 뒷바퀴 50% 역조향 | ❌ 불가 |
| `crab` | 3 | 4륜 같은 방향 → 게처럼 옆으로 평행이동 | ❌ 불가 |
| `spin` | 4 | 제자리 회전 (제로턴) | ✅ 가능 |

`crab`/`spin` 은 STM32가 **고정 조향각**(`CONS(8)`/`CONS(9)`)을 쓰므로 Jetson이
방향(부호)과 속도만 정할 수 있습니다. 연속적인 방향 제어는 `normal` 에서만 됩니다.

### 1.2 가장 중요한 숫자: 최소 회전반경 R_min = 1.643 m

`normal` 모드는 **자동차와 같습니다**. 제자리에서 못 돌고, 어떤 곡선도 R_min 보다
급하게 못 그립니다.

```
기구 상수 (STM32 ALM07.slx CONS 벡터에서):
  wheelbase B    = 1.000 m   (CONS(2))
  track     T    = 0.919 m   (CONS(3))
  rws_ratio      = 0.5       (CONS(1), 뒷바퀴가 앞바퀴의 50% 반대로 조향)
  max_steer      = 30°       (내측 전륜 기구 한계)

        ↓  alm_base_control/scripts/fourwis_encode.py : min_turn_radius()

  R_min = 1.6425 m
  normal 모드 최대 요레이트 = vx / R_min = 0.274 rad/s  @ vx=0.45 m/s
```

> ⚠ **자주 하는 실수**: 뒷바퀴 고정(rws=0)으로 손계산하면 2.079 m 가 나옵니다.
> 이 플랫폼은 `CONS(1)=0.5` 이므로 **1.643 m 가 맞습니다.**

### 1.3 조향각과 곡률은 일대일 대응입니다

이 관계가 이 문서 전체의 뼈대입니다.

```
ICR_y(δ) = half_track + B / (tan δ + tan(rws·δ))      ← 순간회전중심까지의 측방거리
곡률 κ   = 1 / ICR_y = ω / v

  즉  δ (조향각)  ⟷  κ (곡률)  ⟷  ω/v (요레이트/속도 비)
```

- **정방향** (명령 생성): 원하는 `(v, ω)` → 보낼 조향각 `δ`
- **역방향** (관측): 측정된 `(v, ω)` → 실제로 먹은 조향각 `δ`

**두 방향 모두 같은 함수 하나로 됩니다.** `fourwis_encode.solve_inner_front_steer()`
가 이미 구현돼 있습니다 (이분법 60회, 계산량 무시 가능).

### 1.4 속도 한계

```
vx : -0.15 ~ +0.45 m/s   (후진은 전진의 1/3)
vy : ±0.30 m/s           (crab 전용)
wz : ±0.45 rad/s         (spin 상한. normal 에서는 0.274 가 실제 한계)
가속 : 1.0 m/s²  /  1.5 rad/s²
```

---

## 2. 전체 파이프라인 — 목표 pose 가 바퀴까지 가는 길

```
 [사용자] RViz 또는 WebUI 에서 목표 pose 지정
    │
    │  ① 전역 경로계획 (1 Hz)
    ▼
 ComputePathToPose ─── SmacPlannerHybrid (Hybrid-A*, Reeds-Shepp)
    │                   R_min=1.643 을 지키는 모션 프리미티브만 이어붙임
    │                   → 나오는 경로가 처음부터 실현가능
    │
    │  ② 경로 마감
    ▼
 SmoothPath ────────── ConstrainedSmoother (Ceres 최적화)
    │                   프리미티브 이음매 · 각도 양자화(5°) 자국 제거
    │                   실패해도 원본 경로로 주행 (Fallback + AlwaysSuccess)
    │
    │  ③ 경로 추종 (20 Hz)
    ▼
 FollowPath ────────── MPPIController (샘플링 기반 MPC)
    │                   1600개 궤적을 2.80 s 앞까지 굴려보고 비용 최소 선택
    │                   → /cmd_vel_nav  (geometry_msgs/Twist)
    │
    │  ④ 속도 평활화 (20 Hz)
    ▼
 velocity_smoother ─── 가속 제한. scale_velocities=True 로 vx:wz 비 유지
    │                   → /cmd_vel
    │
    │  ⑤ 동작권 중재 (50 Hz)
    ▼
 cmd_arbiter ───────── 자율(/cmd_vel) vs 텔레옵(/cmd_vel_teleop) 중 하나만 통과
    │                   → /cmd_vel_mux
    │
    │  ⑥ 모드 해석 + 안전 게이팅 + 4WIS 인코딩 (50 Hz)
    ▼
 command_manager ───── auto → normal/spin/crab 선택
    │                   |wz| ≤ |vx|/R_min 조향 한계 클램프
    │                   e-stop / timeout / MCU fault / odom 워치독
    │                   twist → (steer_deg, speed_rpm, mode_id)
    │                   → /mcu/command  (alm_msgs/McuCommand)
    │
    │  ⑦ 전송 (순수 전송 계층, 기구학 없음)
    ▼
 mcu_bridge ────────── UART 115200, 16바이트 프레임, CRC16-CCITT
    │                   → STM32
    ▼
 [STM32] FourWIS_DrivingAlgorithm 이 바퀴별 역기구학 수행
```

### 각 단계 상세

#### ① SmacPlannerHybrid (전역 플래너)

`alm_navigation/config/nav2.yaml` → `planner_server.GridBased`

상태공간을 `(x, y, yaw)` 로 놓고 **R_min 을 만족하는 모션 프리미티브만** 이어붙여
탐색합니다. 격자 A*(SmacPlanner2D)와의 차이:

| | SmacPlanner2D | SmacPlannerHybrid |
|---|---|---|
| 로봇을 뭘로 보나 | **점** | SE(2) footprint |
| 경로 곡률 | 제약 없음 (90° 꺾인 경로도 냄) | **R_min 보장** |
| 경로 pose 의 yaw | 전부 0 | **전 구간 유효** |
| 목표 자세 | 무시 | **지킴** |
| 탐색 비용 | 낮음 | 높음 (yaw 축 추가) |

주요 파라미터:
```yaml
minimum_turning_radius: 1.643      # fourwis_encode 유도값과 반드시 일치
motion_model_for_search: REEDS_SHEPP  # 후진 허용 (좁은 곳 전환용)
reverse_penalty: 3.0               # = 실제 속도비 0.45/0.15
angle_quantization_bins: 72        # 5°/bin
max_planning_time: 2.0
lookup_table_size: 10.0            # ↑하면 기동 지연 급증 (20.0 → 8.19 s)
```

#### ② ConstrainedSmoother (스무더)

Hybrid-A* 경로가 이미 실현가능하므로 **'구제'가 아니라 '마감'** 역할입니다.
`minimum_turning_radius: 1.643` 이 플래너와 같아야 합니다 (다르면 스무더가 플래너
결정을 되돌림). `keep_start_orientation`/`keep_goal_orientation` 이 `true` 인 것도
Hybrid-A* 의 시작/목표 자세가 '진짜'이기 때문입니다.

Nav2 기본 BT에는 `SmoothPath` 노드가 없어서 커스텀 BT가 필요합니다:
`alm_navigation/behavior_trees/navigate_*_w_smoothing.xml`
→ `navigation.launch.py` 의 `RewrittenYaml` 이 절대경로를 주입합니다.

#### ③ MPPIController (지역 제어기)

```yaml
time_steps: 56, model_dt: 0.05    # 지평선 2.80 s
batch_size: 1600                  # 궤적 샘플 수
motion_model: "DiffDrive"         # ★ Ackermann 아님 — 아래 설명
critics: 8종
```

> **왜 DiffDrive 인가?** 이 플랫폼은 `spin` 모드에서 제자리 회전이 됩니다.
> MPPI를 Ackermann(R_min 구속)으로 묶으면 제자리 회전을 아예 계획하지 못해
> 자세 정렬을 전부 Nav2 Spin 리커버리에 떠넘기게 됩니다.
> 경로의 R_min 준수는 이미 Hybrid-A* 가 보장하므로 MPPI 는 자유롭게 두고,
> **기구 한계는 `command_manager` 가 최종 책임집니다.**

`PathAlignCritic.use_path_orientations: true` 가 Hybrid-A* 전환의 실질 이득입니다.
경로의 모든 pose 에 유효한 yaw 가 있어야 켤 수 있습니다.

#### ⑥ command_manager (마지막 관문)

50 Hz 틱마다 순서대로:

```
1. 모드 해석      auto → normal/spin/crab (히스테리시스 상태머신)
2. 모드별 제약    spin: 병진 0 / crab: wz=0 / normal: vy=0
3. 속도 클램프    vx[-0.15, 0.45], vy±0.30, wz±0.8
4. 조향 한계      |wz| ≤ |vx|/R_min,  |vx|<0.03 이면 wz=0
5. 하드 정지      e-stop 래치 / cmd timeout 0.5s / MCU fault / odom 워치독 0.5s
6. 가속 제한      1.0 m/s², 1.5 rad/s²
7. 조향 한계 재적용  (가속 램프에서 wz/vx 비가 깨지므로 한 번 더)
8. 4WIS 인코딩    twist → (steer_deg, speed_rpm, mode_id)
```

#### ⑦ UART 프로토콜

```
0xAA 0x55 | msg_type(1) | length(1) | payload | crc16(2, big-endian)

Command (Jetson→STM32, 10 B):  <ffBB  = steer_deg, speed_rpm, mode, flags
State   (STM32→Jetson, 63 B):  <I14fBH
```

**부호 반전 주의**: STM32 내부에서 `normalDriveMode(-steer_deg, ...)` 로 호출되므로
**좌회전(wz>0)에 음수 조향각**을 보냅니다.

**`speed_rpm` 의 의미**: 차체 속도가 아니라 **내측 전륜 rpm** 입니다.

---

## 3. 지금 무엇이 닫혀 있고 무엇이 열려 있는가

이 문서의 핵심 표입니다.

| 피드백 종류 | 신호 | 소비자 | 상태 |
|---|---|---|---|
| **위치·자세** | FAST-LIO `/Odometry.pose` | MPPI (20 Hz 재최적화), 전역 재계획 (1 Hz) | ✅ **닫힘** |
| **속도·요레이트** | `/Odometry.twist` | MPPI `robot_speed` | ❌ **끊김 — 항상 0** |
| **실제 조향각** | `/mcu/state.steer_angle` | (웹 UI 표시 전용) | ❌ **제어에 미사용** |

```
      ┌──────────────────── 위치 피드백 (살아있음) ─────────────────┐
      │                                                             │
      ▼                                                             │
  [MPPI] ──▶ ω_des ──▶ [command_manager] ──▶ δ ──▶ [STM32] ──▶ 로봇 ─┤
                          개루프 변환                                 │
                                                                     │
             ✗ ω_actual ─────── 끊김 (/Odometry.twist = 0) ──────────┘
             ✗ δ_actual ─────── 끊김 (표시용으로만 발행)
```

---

## 4. 발견된 문제 — 상세

### 🔴 문제 1: `/Odometry.twist` 가 항상 0입니다

**증상**: MPPI가 받는 `robot_speed` 가 언제나 (0, 0, 0). 즉 MPPI는 매 틱 "로봇이
정지해 있다"고 가정하고 궤적을 굴립니다.

**원인**: FAST-LIO의 `publish_odometry()` 가 `pose` 만 채웁니다.

```
파일: ALM_auto_ws/src/thirdparty/Fast-LIO2-Localization/FAST_LIO/src/laserMapping.cpp
       publish_odometry()  (981~1017행)

  set_posestamp(odomAftMapped.pose);   ← pose 만 채움
  ...
  odomAftMapped = odom_to_base_msg;    ← send_odom_base_tf 경로에서는
                                          아예 새 메시지로 교체 (twist 기본값 0)
```

**확정 근거**: `laserMapping.cpp` 전체에서 문자열 `twist` 가 **0회** 등장합니다.

**어이없는 부분**: FAST-LIO의 iEKF 상태벡터에는 **속도와 자이로 바이어스가 이미
들어 있습니다.**

```cpp
// include/use-ikfom.hpp
MTK_BUILD_MANIFOLD(state_ikfom,
  ((vect3, pos))   ((SO3, rot))          // 위치 · 자세
  ((SO3, offset_R_L_I)) ((vect3, offset_T_L_I))
  ((vect3, vel))                          // ★ 속도 — 이미 추정 중
  ((vect3, bg))                           // ★ 자이로 바이어스 — 이미 추정 중
  ((vect3, ba))    ((S2, grav))
);

// src/IMU_Processing.hpp:282
angvel_last = angvel_avr - imu_state.bg;  // ★ 바이어스 제거된 각속도 — 이미 계산됨
```

**즉 값은 다 있는데 발행만 안 합니다.**

---

### 🔴 문제 2: 조향 피드백이 전혀 없습니다

**증상**: 조향 명령을 보내고 나면, 실제로 몇 도가 꺾였는지 시스템이 모릅니다.

**확인 결과**: STM32는 실제 조향각을 올려보내고 `mcu_bridge` 가
`st.steer_angle = [steer_f, steer_r]` 로 발행합니다. 이걸 소비하는 곳을 전수 검색:

| 소비자 | 용도 |
|---|---|
| `mcu_bridge` 자신 → `/joint_states` | RViz 시각화 |
| `alm-webui-v0.6/src/ingest.js:333` | 웹 UI 게이지 표시 |

**끝입니다.** `command_manager` 는 `/mcu/state` 를 구독하지만 `fault` / `emergency_stop`
만 봅니다 (`command_manager.py` 308~313행). 명령 `steer_deg` 와 실제 `steer_angle` 이
얼마나 벌어졌는지 **감시조차 하지 않습니다.**

**따라오는 위험**: 아래 상수들이 전부 미확정(`##CONFIRM##`)인데, **틀려도 아무도
모릅니다.**

```
wheel_radius_m: 0.103      # 접지 실효 반지름. rpm 환산에 직접 영향
gear_ratio: 1.0            # 모터 rpm / 휠 rpm
max_steer_deg: 30.0        # 실제 기구 한계
straight_angle_deg: 2.0    # CONS(4) 직진 데드밴드
crab_steer_sign / spin_steer_sign
```

예: `gear_ratio` 가 실제로 20이면 명령이 20배 작게 나가는데, 검출 수단이 없습니다.

---

### 🟠 문제 3: 출발 시 조향 명령이 20 ms 만에 0° → 30°

`command_manager` 는 vx/vy/wz 에만 가속 제한을 겁니다. **`steer_deg` 에는 슬루 레이트
제한이 없습니다.**

틱 루프를 그대로 재현한 결과 — **`vx=0.45, wz=0.20` (R=2.25 m, 완전히 실현가능한 요청)**:

```
 t[s]    vx      wz     steer[deg]
 0.00   0.020   0.000      0.000
 0.02   0.040   0.024    -30.000   ← 20 ms 만에 0° → 30°  (슬루 1500 deg/s)
 0.04   0.060   0.037    -30.000
 ...                              ← vx < 0.329 구간 내내 풀락 포화
 0.32   0.340   0.200    -28.788
 0.44   0.450   0.200    -20.631   ← 0.44 s 후에야 목표값 도달
```

**원인은 구조적입니다.** `|wz| ≤ |vx|/R_min` 클램프가 걸리면
`R = vx/wz = R_min` 이 되는데, 이는 **정의상 최대 조향각**입니다.

```
출발 시 vx 가 작음  →  wz 한계도 작음  →  조금만 돌려도 클램프 발동
                                        →  조향각 = 정확히 30° (풀락)
```

즉 **모든 출발 선회에서 0.3초간 풀락을 때립니다.** 어떤 조향 모터도 1500 deg/s 를
못 따라가므로, 실제로는 훨씬 덜 꺾인 채로 주행합니다 — 그리고 **얼마나 덜 꺾였는지
아무도 모릅니다** (문제 2).

---

### 🟠 문제 4: 동적 장애물 반응이 최악 4.3 초 / 1.94 m

#### 되어 있는 것

- **전역 costmap에도 `obstacle_layer` 가 붙어 있습니다.** 맵에 없는 장애물이 전역
  costmap에 마킹되므로, 재계획하면 실제로 우회 경로가 나옵니다. (이게 없는 스택도
  많아서 이건 잘 되어 있는 편)
- MPPI `CostCritic.consider_footprint: true`, `collision_cost: 1000000`
- 감지 거리는 충분: local costmap 8×8 m(전방 4 m), 전역은 라이다 15 m

#### 문제점 (a) — MPPI는 회피 기동을 완주할 수 없습니다

지평선 2.8 s 안에 낼 수 있는 횡변위를 R_min 으로 계산하면:

| vx | 지평선 거리 | 단일 호 횡변위 | S자(원경로 복귀) |
|---|---|---|---|
| 0.45 | 1.26 m | 0.460 m | **0.239 m** |
| 0.30 | 0.84 m | 0.210 m | **0.107 m** |
| 0.20 | 0.56 m | 0.095 m | **0.048 m** |

장애물을 비켜 가려면 차체 반폭 0.53 m + 여유 = **최소 0.6~1.0 m** 필요.
→ **MPPI 단독으로는 물리적으로 못 피합니다.**

실제 동작은 "MPPI가 감속/정지 → 전역 재계획이 우회로를 생성" 이고, **회피의 주체는
로컬이 아니라 글로벌**입니다. 자동차형 로봇에서는 이 자체가 잘못된 건 아닙니다.
문제는 아래 지연입니다.

#### 문제점 (b) — 재계획이 순수 시간 구동입니다

BT에 `IsPathValid` 나 `PathExpiringTimer` 가 **없습니다**. 경로 위에 장애물이 생겨도
다음 `RateController` 틱을 기다립니다.

```
lidar 10 Hz               0.10 s → 0.05 m
local costmap 5 Hz        0.20 s → 0.09 m
global costmap 1 Hz       1.00 s → 0.45 m
BT RateController 1 Hz    1.00 s → 0.45 m
max_planning_time         2.00 s → 0.90 m
─────────────────────────────────────────
최악 합계                 4.30 s → 1.94 m 주행 후에야 새 경로

참고: 제동거리는 0.45²/(2×1.5) = 0.068 m — 제동은 여유. 문제는 '결정'
```

#### 문제점 (c) — `navigate_through_poses` 는 3초입니다

`RateController hz="0.333"` → 웨이포인트 주행 중 최악 지연 약 6 s / 2.7 m.

#### 문제점 (d) — 장애물 예측이 없습니다

`ObstacleLayer` 는 마킹/레이트레이싱만 하고 시간·속도 개념이 없습니다. MPPI는 2.8 s
지평선을 **정지한 costmap 스냅샷**에 대고 평가합니다. 1.2 m/s로 걷는 사람은 그 사이
3.4 m 이동합니다.

---

### 🟡 문제 5: 미확정 상수

`base_control.yaml` 에 `##CONFIRM##` 로 표시된 값들. 문제 2와 맞물려 **틀려도 검출
수단이 없다**는 점이 본질입니다.

---

## 5. 왜 이렇게 됐는가 — 설계 의도는 옳았습니다

오해를 막기 위해 적습니다. **이 코드베이스는 잘 만들어져 있습니다.**

- 모든 숫자가 기구 상수에서 유도되고, 하드코딩된 상수가 없습니다
- `fourwis_encode.py` 가 단일 진실 공급원이고, `nav2_kinematic_check.py` 라는
  정합성 검사기까지 있습니다
- 주석에 "왜 이 값인가"와 "무엇을 시도했다가 왜 뺐는가"가 기록돼 있습니다
- e-stop 래치, 동작권 중재, 3중 워치독 등 안전 설계가 촘촘합니다

문제는 **"계획을 실현가능하게 만든다"는 축에는 온 힘을 쏟았지만, "실행 결과를
되먹인다"는 축은 아직 손대지 않았다**는 것입니다. 문서(`docs/nav2_planning.md` §5)에도
"실제 경로 생성 / 주행: ❌ 미검증" 이라고 정직하게 적혀 있습니다.

---

## 6. 해결 방향

### 6.1 즉시 — 파라미터/BT만 (재빌드 최소, 리스크 낮음)

| # | 조치 | 효과 |
|---|---|---|
| 1 | **BT에 `IsPathValid` 추가** (`RateController` 밖 `ReactiveSequence`) | 1 Hz 대기(0.45 m) 제거. `plugin_lib_names` 에 이미 등록됨 |
| 2 | `global_costmap.update_frequency: 1.0 → 3.0` | 0.45 m → 0.15 m |
| 3 | `navigate_through_poses` 의 `hz="0.333" → "1.0"` | 3 s → 1 s |
| 4 | `max_planning_time: 2.0 → 1.0` + `downsample_costmap: true` | 0.90 m → 0.45 m |
| 5 | 동적 환경에서 `vx_max: 0.45 → 0.30` | 지연 거리 1.94 → 1.29 m |

**1~4번만으로 최악 지연 4.30 s / 1.94 m → 약 1.4 s / 0.42 m.**

### 6.2 단기 — 피드백 선 잇기

#### (A) `/Odometry.twist` 채우기 — 최우선

값은 이미 다 계산돼 있으므로 패치는 3줄입니다.

```cpp
// laserMapping.cpp publish_odometry(), set_posestamp 직후
V3D vel_body = state_point.rot.conjugate() * state_point.vel;   // world → body
odomAftMapped.twist.twist.linear.x  = vel_body(0);
odomAftMapped.twist.twist.linear.y  = vel_body(1);
odomAftMapped.twist.twist.angular.z = p_imu->angvel_last(2);    // 이미 bias 제거됨
```

**프레임 확인 완료**: `base_link → livox_frame` 정적 변환이 `x=0, y=0, z=0.5,
rpy=0` 입니다 (`alm_sensors/launch/lidar.launch.py`).
- 회전이 단위행렬 → **ω_z 가 두 프레임에서 동일**
- 레버암이 순수 수직 → 평면 운동에서 `ω × r = 0` → **선속도 보정 불필요**

> ⚠ 이 마운트는 `##TODO## 실측` 상태입니다. 실측 후 x나 y가 0이 아니게 되면
> 레버암 보정이 필요합니다 (r=0.5 m, ω=0.27 rad/s → 0.14 m/s, 무시 못 함).

> ⚠ `thirdparty` 서브모듈이라 업스트림과 갈라집니다. 대안: FAST-LIO를 안 건드리고
> `/livox/imu` + pose 미분으로 twist를 채우는 릴레이 노드 (정확도는 떨어지지만
> 서브모듈은 깨끗하게 유지).

이 패치 하나가 **MPPI 속도 피드백을 되살립니다.**

#### (B) 조향 관측기 — 엔코더 없이 실제 조향각 알아내기

`/mcu/state.steer_angle` 을 못 믿거나 안 쓰더라도, **주행 데이터만으로 실효 조향각을
역산할 수 있습니다.**

```python
# δ_actual = solve_inner_front_steer(v_meas, ω_meas, params)
#   → 명령 생성에 쓰는 함수를 그대로, 입력만 측정값으로
```

왕복 검증 결과 오차 0:
```
v=0.45 w=0.200 : 명령 steer=20.63°  역추정=20.63°
v=0.30 w=0.150 : 명령 steer=23.71°  역추정=23.71°
```

**정확도** (자이로 잡음 σ_ω 별, 추정 조향각 1σ):

| v [m/s] | σ_ω=0.005 | σ_ω=0.01 | σ_ω=0.02 |
|---|---|---|---|
| 0.45 | ±0.58° | **±1.16°** | ±2.34° |
| 0.30 | ±0.87° | ±1.75° | ±3.53° |
| 0.20 | ±1.31° | ±2.63° | ±5.36° |
| 0.10 | ±2.63° | ±5.36° | ±11.03° |

**순항속도(0.3~0.45)에서 ±1~2°** — 감시용으로 충분합니다.

**⚠ 원리적 한계 — 저속에서는 관측 불가**

`κ = ω/v` 이므로 `v → 0` 이면 어떤 조향각도 `ω = 0` 을 만듭니다.

```
v=0.45 → 조향 0~30° 전체가 ω 0~15.7 deg/s 에 대응  (1° 당 9.13 mrad/s)
v=0.10 → 0~3.5 deg/s                              (1° 당 2.03 mrad/s)
v=0.03 → 0~1.0 deg/s                              (1° 당 0.61 mrad/s)
```

**정지 상태에서는 바퀴가 30° 꺾여 있어도 알 수 없습니다.**
→ `v > 0.15~0.2` 에서만 신뢰. **하필 문제 3(출발 시 풀락)이 이 사각지대입니다.**

**⚠ 지연 정렬 필수**

`δ_actual(t)` 는 `δ_cmd(t − τ)` 의 결과입니다. 정렬 안 하면 **지연을 간극으로
오독**합니다. 뒤집으면 이득: **`δ_cmd` 와 `δ_actual` 을 상호상관하면 τ와 시상수를
실측**할 수 있습니다. 지금 조향 서보 응답 속도를 아무도 모르는데, 이걸로 알 수 있습니다.

#### (C) 부수 효과 — `##CONFIRM##` 상수가 전부 풀립니다

| 미확정 값 | 확정 방법 |
|---|---|
| `wheel_radius_m`, `gear_ratio` | 직진: FAST-LIO의 v [m/s] ÷ 명령 `speed_rpm` |
| `max_steer_deg` (실제 R_min) | 풀락 선회: κ=ω/v 측정 → **R_actual = 1/κ** |
| `rws_ratio` | 위 R_actual에 녹아 있음 (rws=0이면 2.079 m 가 나와야 함 — 구분됨) |
| `straight_angle_deg` | 조향각을 올리며 ω가 처음 반응하는 지점 |
| `spin_steer_sign` / `crab_steer_sign` | spin 시 ω 부호 / crab 시 횡방향 이동 부호 |

### 6.3 중기 — 제어 루프 추가

#### 올바른 3층 구조

```
δ_cmd = δ_ff(ω_des, v)              ← 기존 fourwis_encode (그대로)
      + K(v) · (ω_des − ω_meas)     ← 요레이트 내부루프, 50 Hz, 비례만
      + δ_trim                       ← 아주 느린 적분 (τ ≈ 5~10 s)

조건:
  ω_meas   : 자이로 z − state_point.bg   (헤딩 미분 아님)
  내부루프 : |v| > 0.15 에서만 활성, K(v) 상한 클램프
  δ_trim   : |dδ_cmd/dt| < 5 deg/s AND |v| > 0.25 인 구간에서만 적분, ±5° 클램프
  최종     : 슬루 제한 후 ICR 역산으로 wz 재계산 → McuCommand.cmd_vel 정합 유지
```

#### 조향 슬루 제한 (문제 3의 답)

그냥 `steer_deg` 를 자르면 이 코드베이스가 지키려는 "cmd_vel과 steer_deg가 같은
이야기를 한다"는 원칙이 깨집니다. 올바른 순서:

```python
steer_cmd = slew_limit(steer_target, prev, max_steer_rate_deg_s * dt)
wz_actual = sign * abs(vx) / params._icr_y(radians(steer_cmd))   # 조향각→wz 역산
McuCommand.cmd_vel.angular.z = wz_actual                          # 두 필드 재정합
```

`max_steer_rate_deg_s` 는 (B)의 상호상관으로 실측해야 합니다.

#### 게인 스케줄링 필수

내부루프 제어권한 `∂ω/∂δ` [(rad/s)/deg]:

| v | δ=5° | δ=15° | δ=25° | δ=30° |
|---|---|---|---|---|
| 0.45 | 0.01054 | 0.00886 | 0.00790 | 0.00762 |
| 0.20 | 0.00468 | 0.00394 | 0.00351 | 0.00338 |
| 0.05 | 0.00117 | 0.00098 | 0.00088 | 0.00085 |

**속도에 거의 비례하고 최대 12배 차이**납니다. 고정 게인은 고속에서 발진하거나
저속에서 무력해집니다. `K ∝ 1/v` 스케줄 + 상한 클램프.

### 6.4 ⛔ 하지 말아야 할 것

이 세 가지는 **직관적으로는 맞아 보이지만 틀립니다.**

#### (1) 헤딩 오차를 조향에 직접 먹이지 말 것

MPPI가 **이미** 하고 있습니다. `PathAlignCritic.use_path_orientations: true`
(가중치 14.0, 최대값)가 경로 yaw와 실제 헤딩 차이에 직접 비용을 매깁니다.
`command_manager` 에 또 넣으면 **같은 측정치·같은 오차를 두고 두 제어기가 싸웁니다.**

#### (2) 헤딩(θ) 말고 요레이트(ω)를 쓸 것

세 가지 이유:

1. **헤딩은 이미 요레이트 오차의 적분**입니다 — `θ_err = ∫(ω_des − ω_act)dt`.
   여기 또 적분을 걸면 이중 적분 → 위상 지연 → 발진.
2. **map 프레임 헤딩은 계단 점프합니다.** `map→odom` 은 `/icp_result`
   (FPFH+TEASER++ → GICP)로 갱신되고 수용 기준이 `consistency_rotation_deg: 5.0`
   입니다. 재측위가 들어오면 **헤딩이 최대 5° 순간 점프**하고, 그게 조향 킥이 됩니다.
3. **자이로 요레이트는 점프가 없습니다.** 관성 직접 측정, 200 Hz, 바이어스도
   FAST-LIO가 이미 추정합니다.

#### (3) 지연에 적분 보정을 걸지 말 것

간극에는 두 성분이 있고 대책이 정반대입니다.

| 성분 | 원인 | 적분 보정 |
|---|---|---|
| **편향 (bias)** | `max_steer_deg`·`rws_ratio`·기계 정렬 오차 | ✅ 효과적 |
| **지연 (lag)** | 조향 서보 응답 시간 | ❌ 발진 유발 |

구분 기준 (조향 지연 τ=0.25 s 가정):

```
δ_cmd 변화율   0 deg/s → 지연으로 생기는 간극 =  0.00°  ← 여기 남는 것만 '진짜 편향'
δ_cmd 변화율  20 deg/s → 지연으로 생기는 간극 =  5.00°  ← 이건 그냥 지연
δ_cmd 변화율  60 deg/s → 지연으로 생기는 간극 = 15.00°
```

**적분은 `|dδ_cmd/dt| ≈ 0` 인 정상 주행 구간에서만.**
그리고 문제 3(출발 시 0→30°)은 **지연 쪽**이라 적분으로는 안 고쳐집니다.

**참고 — 편향 신호는 잘 보입니다.** 조향이 3° 부족하면:
```
v=0.45 → 헤딩이 1.45°/s 로 벌어짐 (3초면 4.4°)
v=0.30 → 0.97°/s
```

### 6.5 권장 진행 순서

```
1. /Odometry.twist 채우기          ← 공짜, 리스크 없음, 효과 큼
2. BT IsPathValid + costmap 주기    ← 파라미터, 동적 장애물 반응 3배 개선
3. 조향 관측기 (로깅 전용)          ← 제어 경로 안 건드림. 편향/지연 비율 실측
        ↓  여기서 데이터를 보고 결정
4a. 편향이 지배적 → δ_trim 적분 추가 (느리고 클램프, 안전)
4b. 지연이 지배적 → 조향 슬루 제한 (개루프, 적분 금지)
5. ##CONFIRM## 상수 확정 → base_control.yaml 반영
```

> **3번 없이 4번을 하면 안 됩니다.** 어떤 성분이 얼마나 되는지 모르는 상태에서
> 제어기를 설계하면 잘못된 대책을 고르게 됩니다.

---

## 7. 재현 / 검증 명령

```bash
# 기구학 상수 자체 검증 (ROS 불필요)
python3 ALM_auto_ws/src/alm_base_control/scripts/fourwis_encode.py
#  → [normal] 역산<->정방향 일치, [frame] 문서 예시 프레임 바이트 일치 등 7항목

# nav2.yaml 과 base_control.yaml 정합성 검사
ros2 run alm_navigation nav2_kinematic_check.py
#  → 종료코드 0 = 일치, 1 = 어긋남

# 하드웨어 없이 경로 생성까지 확인
ros2 launch alm_bringup webui_dev.launch.py
ros2 run alm_navigation pcd_replay.py

# 조향 명령 확인
ros2 topic echo /mcu/command --field steer_deg
ros2 topic echo /mcu/state  --field steer_angle    # 지금은 제어에 미사용

# twist 가 0인지 직접 확인 (문제 1)
ros2 topic echo /Odometry --field twist.twist
```

---

## 8. 용어집

| 용어 | 뜻 |
|---|---|
| **R_min** | 최소 회전반경. 이 플랫폼 1.643 m. 이보다 급한 선회 불가 |
| **ICR** | 순간회전중심 (Instantaneous Center of Rotation) |
| **κ (곡률)** | `1/R = ω/v`. 조향각과 일대일 대응 |
| **RWS** | Rear Wheel Steering. 뒷바퀴 보조조향. 여기선 앞바퀴의 50% 역방향 |
| **footprint** | costmap 충돌검사용 차체 외곽. x[-0.65, 0.72], y[±0.53] |
| **inscribed / circumscribed** | 내접(0.530 m) / 외접(0.894 m) 반경. `inflation_radius` 는 외접보다 커야 함 |
| **BT** | Behavior Tree. Nav2가 플래너/스무더/제어기를 엮는 방식 |
| **critic** | MPPI가 궤적을 평가하는 비용 함수 조각 |
| **iEKF** | Iterated EKF. FAST-LIO의 상태 추정기 |
| **개루프 / 폐루프** | 결과를 되먹이지 않음 / 되먹임 |

---

## 9. 파일 지도

### 경로계획
```
alm_navigation/config/nav2.yaml                       모든 Nav2 파라미터 (유도 근거를 주석에 기록)
alm_navigation/behavior_trees/navigate_*_w_smoothing.xml   SmoothPath 를 끼운 커스텀 BT
alm_navigation/launch/navigation.launch.py            BT 절대경로 주입 (RewrittenYaml)
alm_navigation/scripts/nav2_kinematic_check.py        정합성 검사기
```

### 제어
```
alm_base_control/scripts/fourwis_encode.py    ★ 기구학 단일 진실 공급원 (R_min, twist→조향각)
alm_base_control/scripts/command_manager.py     모드 해석 · 안전 게이팅 · 조향 한계
alm_base_control/scripts/cmd_arbiter.py         자율/텔레옵 동작권 중재
alm_base_control/config/base_control.yaml       기구 상수 (STM32 CONS 와 일치 필수)
```

### 하드웨어 인터페이스
```
alm_mcu_interface/scripts/mcu_bridge.py         UART 전송 계층 (기구학 없음)
alm_mcu_interface/docs/uart_protocol.md          프레임 규격 v2
alm_bringup/scripts/fake_mcu.py                  하드웨어 없이 테스트
```

### 측위
```
alm_navigation/launch/localization.launch.py     FAST-LIO + ICP 재측위
alm_navigation/config/fastlio_relocalization.yaml
thirdparty/Fast-LIO2-Localization/FAST_LIO/src/laserMapping.cpp   ← 문제 1의 패치 지점
thirdparty/Fast-LIO2-Localization/FAST_LIO/include/use-ikfom.hpp  ← iEKF 상태벡터
```

### 관련 문서
```
docs/nav2_planning.md          경로계획 설계 근거 (왜 Hybrid-A* 인가)
docs/control_arbitration.md    /cmd_vel 이후의 경로
docs/uart.md                   UART 개요
docs/TODO.md                   미해결 과제
```

---

## 10. 검증 상태

| 항목 | 상태 |
|---|---|
| 문제 1 (`/Odometry.twist` = 0) | ✅ 코드 정독으로 확정 (`twist` 문자열 0회) |
| 문제 2 (조향 피드백 없음) | ✅ 소비자 전수 검색으로 확정 |
| 문제 3 (조향 슬루 1500 deg/s) | ✅ 틱 루프 재현 시뮬레이션 |
| 문제 4 (지연 예산) | ⚠️ 파라미터 기반 계산. 실측 아님 |
| 회피 횡변위 계산 | ⚠️ 기구학 계산. 실주행 미확인 |
| 조향 관측기 정확도 | ⚠️ 가정한 자이로 잡음 기준. **실제 σ_ω 실측 필요** |
| 조향 지연 τ = 0.25 s | ❌ **순수 가정값.** 실측 필요 (관측기로 측정 가능) |
| 실제 경로 생성 / 주행 | ❌ **미검증** (`docs/nav2_planning.md` §5와 동일) |
