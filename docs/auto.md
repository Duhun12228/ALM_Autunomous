# Nav2 자율주행 스택 개편 정리 (SmacPlanner2D + ConstrainedSmoother + MPPI)

> 이 문서는 ALM 로봇의 **Nav2 경로계획/추종 스택을 교체한 작업**을 처음 보는 사람이
> 이해할 수 있도록 정리한 것입니다. Nav2를 잘 몰라도 위에서부터 순서대로 읽으면 됩니다.
>
> - 작업 대상: `ALM_auto_ws/src/alm_navigation`
> - 환경: ROS 2 **Humble**, Nav2 **1.1.20**
> - 관련 문서: [README.md](../README.md) · [OPERATION_GUIDE.md](OPERATION_GUIDE.md)

---

## 0. 30초 요약

| | 이전 | 이후 |
|---|---|---|
| **전역 경로계획** | NavFn (A*/Dijkstra) | **SmacPlanner2D** (2D A*) |
| **경로 평활화** | 없음 | **ConstrainedSmoother** (Ceres 최적화) |
| **지역 제어** | DWB (Dynamic Window) | **MPPI** (샘플링 기반 MPC) |
| **Behavior Tree** | Nav2 기본 트리 | **커스텀 트리** (SmoothPath 삽입) |
| **local costmap** | 5 × 5 m | 8 × 8 m |
| **inflation_radius** | 0.45 / 0.55 m | 1.00 / 1.10 m |

바꾼 파일은 **7개**, 새로 만든 파일은 **3개**입니다. 자세한 목록은 [6장](#6-변경-파일-전체-목록)에.

---

## 1. 배경 — Nav2는 어떻게 생겼나

Nav2는 "목표 지점을 찍으면 `/cmd_vel`(속도 명령)을 만들어 주는" ROS 2 표준 내비게이션
프레임워크입니다. 하나의 프로그램이 아니라 **역할별로 나뉜 여러 서버**로 되어 있고,
그 서버들을 **Behavior Tree(BT)** 라는 시나리오 파일이 순서대로 호출합니다.

```
[RViz 에서 목표 찍기]
        │
        ▼
   bt_navigator ────────── BT xml 을 읽어서 아래 서버들을 순서대로 호출
        │
        ├─▶ planner_server     "출발점 → 목표점 전역 경로 만들어줘"   → /plan
        ├─▶ smoother_server    "그 경로 좀 매끄럽게 다듬어줘"          → 평활화된 /plan
        └─▶ controller_server  "이 경로 따라가는 속도명령 계속 줘"     → /cmd_vel
                                                                           │
                                                                           ▼
                                                              command_manager → STM32
```

핵심은 **각 서버가 "플러그인"을 갈아끼우는 구조**라는 점입니다.
`planner_server`라는 껍데기는 그대로 두고 그 안의 알고리즘만 NavFn → SmacPlanner2D로
교체하는 식입니다. 이번 작업이 딱 그것입니다.

### 이 로봇의 특징 (설계 판단의 근거)

`alm_description/urdf/alm_robot.urdf.xacro`의 CAD 실측값입니다.

| 항목 | 값 | 의미 |
|---|---|---|
| footprint | x `[-0.65, +0.72]`, y `[±0.52]` | 전장 **1.37 m** × 전폭 **1.04 m** |
| inscribed radius | **0.52 m** | footprint 안에 들어가는 최대 원 |
| circumscribed radius | **0.888 m** | footprint를 감싸는 최소 원 (√(0.72²+0.52²)) |
| 휠베이스 | 0.9116 m | 앞축 +0.6106, 뒷축 −0.3010 |
| 구동방식 | 4WIS (4륜 독립조향) | normal / crab / spin 모드 |
| 속도 한계 | vx `[−0.15, 0.45]` m/s, wz `±0.8` rad/s | |

**중요**: 이 로봇은 원형과 거리가 먼 **크고 긴 직사각형**입니다(1.37 × 1.04 m).
원형 근사로 충돌검사를 하면 지나치게 보수적이 되고, 반대로 팽창 반경을 작게 잡으면
차체 모서리가 벽을 긁습니다. 아래 costmap 변경([5장](#5-costmap-변경--이건-선택이-아니라-필수였습니다))이
여기서 나옵니다.

또 `command_manager`가 `/cmd_vel`을 보고 normal↔spin을 자동 전환하므로,
Nav2 입장에서 이 로봇은 **"제자리 회전이 가능한 차동구동(DiffDrive)"** 처럼 보입니다.
측면 이동(crab)은 기본 비활성이라 Nav2는 `vy`를 쓰지 않습니다.

---

## 2. 무엇을 왜 바꿨나

### 2-1.게 전역 경로계획: NavFn → SmacPlanner2D

**NavFn**은 Nav2의 가장 오래된 기본 플래너입니다. 동작은 하지만 격자 위 최단경로를
그대로 뱉어서 **경로가 각지고 계단처럼 나옵니다**.

**SmacPlanner2D**는 같은 2D 격자 A*지만:

- **8방향(8-connected) 탐색** — NavFn보다 대각선 이동이 자연스러움
- **`cost_travel_multiplier`** — "거리는 좀 멀어도 벽에서 떨어져 가기"를 조절 가능
- **내장 경량 스무더** — 격자 계단 현상을 1차로 제거
- **`max_planning_time`** — 시간 초과 시 tolerance 안의 최선해를 반환 (멈추지 않음)

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0     # BT가 1 Hz로 재계획하므로 맞춤 (안 맞추면 경고 스팸)
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.50                   # 목표에 정확히 못 붙을 때 허용 반경 (m)
      allow_unknown: true               # 미탐사 영역 통과 허용
      max_planning_time: 2.0            # 초과 시 tolerance 내 최선해 반환
      cost_travel_multiplier: 2.0       # ↑ 벽에서 더 떨어져 돌아감 / ↓ 최단거리 우선
      use_final_approach_orientation: true
      smoother:                         # 내장 경량 스무더 (계단현상 제거용)
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        do_refinement: true
```

> ⚠ **`use_final_approach_orientation: true`의 의미** — 아래 [9-1](#9-1-목표-yaw가-무시됩니다)에서
> 자세히 설명합니다. 이 한 줄이 이번 개편에서 가장 중요한 트레이드오프입니다.

### 2-2. 경로 평활화: (없음) → ConstrainedSmoother

이전엔 평활화 단계 자체가 없었습니다. Nav2에 `smoother_server`가 떠 있긴 했지만
**아무도 호출하지 않는 상태**였습니다 (이유는 [4장](#4-behavior-tree--가장-놓치기-쉬운-부분)).

**ConstrainedSmoother**는 Ceres(Google의 비선형 최소제곱 라이브러리)로 아래 네 항목의
가중합을 최소화해서 경로를 다듬습니다:

| 항 | 파라미터 | 뜻 |
|---|---|---|
| 매끄러움 | `w_smooth` | 인접 점들이 급격히 꺾이지 않게 |
| 곡률 제한 | `w_curve` | `minimum_turning_radius`보다 급한 곡률에 페널티 |
| 원경로 유지 | `w_dist` | 원래 경로에서 멀어지지 않게 (0이면 미사용) |
| 장애물 회피 | `w_cost` | costmap 비용이 높은 곳을 피하게 |

```yaml
smoother_server:
  ros__parameters:
    costmap_topic: "global_costmap/costmap_raw"
    footprint_topic: "global_costmap/published_footprint"
    robot_base_frame: base_link
    transform_tolerance: 0.3
    smoother_plugins: ["SmoothPath"]
    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"

      # ── SmacPlanner2D 경로엔 유효한 자세(yaw)가 없어서 끄는 것들 ──
      reversing_enabled: false        # 후진구간 판정 불가 (자세가 없으니)
      keep_start_orientation: false   # 시작 자세가 무의미한 값이라 고정하면 안 됨
      keep_goal_orientation: true     # 마지막 pose만은 planner가 진행방향으로 채워둠 → 유지

      path_downsampling_factor: 3     # 3점마다 최적화 (속도)
      path_upsampling_factor: 1       # 결과를 베지어 곡선으로 되살릴 배수

      minimum_turning_radius: 0.60    # ##TODO## 실제 조향각 한계로 검증 필요
      w_curve: 30.0
      w_dist: 0.0                     # 0 = 원경로 추종 강제 안 함 (w_smooth가 형상 지배)
      w_smooth: 15000.0
      w_cost: 0.020                   # ↑ 장애물에서 더 멀어지려 함
      w_cost_cusp_multiplier: 3.0
      cusp_zone_length: 2.5

      # base_link 원점 외에 추가로 costmap 비용을 볼 점 [x, y, weight] 묶음
      # 차체가 길어서(앞 +0.72 / 뒤 −0.65) 앞뒤 끝을 함께 봅니다
      cost_check_points: [0.55, 0.0, 1.0, -0.45, 0.0, 1.0]

      optimizer:
        max_iterations: 70
        gradient_tol: 5.0e-3
        fn_tol: 1.0e-6
        param_tol: 1.0e-15
        linear_solver_type: "SPARSE_NORMAL_CHOLESKY"
```

`reversing_enabled` / `keep_start_orientation`을 왜 껐는지는 [9-1](#9-1-목표-yaw가-무시됩니다)을 보세요.
**한 줄 요약: SmacPlanner2D가 만드는 경로에는 자세 정보가 없기 때문**입니다.

### 2-3. 지역 제어: DWB → MPPI

**DWB(Dynamic Window)** 는 "지금 낼 수 있는 속도 조합을 격자로 나열해 보고 점수가 제일
높은 걸 고르는" 방식입니다. 단순하고 가볍지만 샘플이 격자에 묶여 있어 움직임이 뚝뚝
끊기고, 큰 로봇에서 부드러운 곡선 추종이 잘 안 나옵니다.

**MPPI(Model Predictive Path Integral)** 는 매 주기마다:

1. 현재 제어 시퀀스에 **랜덤 노이즈를 뿌려 수천 개의 궤적을 생성**하고
2. 각 궤적을 **critic(평가함수)들이 점수 매긴 뒤**
3. 점수에 따른 **가중평균**으로 최적 제어를 뽑습니다.

결과가 훨씬 부드럽고, critic을 갈아끼워 행동을 세밀하게 조율할 수 있습니다. 대신 CPU를
많이 씁니다.

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"

  # ── 예측 지평선 = time_steps × model_dt = 40 × 0.05 = 2.0 초 ──
  time_steps: 40
  model_dt: 0.05          # controller_frequency(20 Hz)의 역수와 맞춤
  batch_size: 1600        # 매 주기 굴려보는 궤적 개수
  iteration_count: 1

  motion_model: "DiffDrive"   # command_manager가 normal↔spin을 담당하므로
  vx_max: 0.45
  vx_min: -0.15               # 음수 = 후진 허용
  vy_max: 0.0                 # crab 미사용
  wz_max: 0.8

  vx_std: 0.15                # 탐색 노이즈 표준편차 (제어 범위의 1/3 정도)
  vy_std: 0.0
  wz_std: 0.30

  temperature: 0.3            # ↓ 최고점수 궤적에 몰빵 / ↑ 넓게 평균
  gamma: 0.015
  prune_distance: 2.0         # 경로 중 앞으로 이만큼만 보고 추종
  transform_tolerance: 0.3
  reset_period: 1.0           # 제어가 이만큼 끊기면 제어 시퀀스 초기화
  visualize: false            # true면 RViz에 궤적 표시 (CPU↑, 튜닝할 때만)
```

**critic 구성** — 각각이 궤적에 점수를 매기는 독립 평가자입니다.

| Critic | 역할 | 주요 설정 |
|---|---|---|
| `ConstraintCritic` | 속도 한계 위반 페널티 | `cost_weight: 4.0` |
| `CostCritic` | 장애물/costmap 비용 회피 | **`consider_footprint: true`** |
| `GoalCritic` | 목표점에 가까워지게 | `threshold_to_consider: 1.0` |
| `GoalAngleCritic` | 목표 자세 맞추기 | `threshold_to_consider: 0.5` |
| `PathAlignCritic` | 경로와 나란히 가게 | **`use_path_orientations: false`** |
| `PathFollowCritic` | 경로 앞쪽 점을 향해 전진 | `offset_from_furthest: 5` |
| `PathAngleCritic` | 경로 방향과 헤딩 정렬 | `forward_preference: true` |
| `PreferForwardCritic` | 후진보다 전진 선호 | `cost_weight: 5.0` |

굵게 표시한 두 개가 이 로봇에 맞춘 부분입니다:

- **`CostCritic.consider_footprint: true`** — 1.37 × 1.04 m 직사각형이라 원형 근사와
  차이가 큽니다. 실제 footprint로 충돌검사를 합니다.
  성능 걱정은 안 해도 됩니다: 내부적으로 **중심점 비용이 circumscribed 임계값을 넘을 때만**
  전체 footprint 검사로 넘어가므로, 열린 공간에서는 거의 비용이 들지 않습니다.
- **`PathAlignCritic.use_path_orientations: false`** — SmacPlanner2D 경로엔 유효한 yaw가
  없으므로, 경로의 자세값을 믿지 말라고 알려주는 설정입니다. 켜두면 엉뚱한 방향으로
  정렬하려 듭니다.

---

## 3. 세 가지가 어떻게 이어지나

```
 RViz "Nav2 Goal"
        │
        ▼
┌─ bt_navigator ───────────────────────────────────────────────────────┐
│   behavior_trees/navigate_to_pose_w_smoothing.xml 를 실행            │
│                                                                      │
│   ① ComputePathToPose  ──▶ planner_server (SmacPlanner2D)            │
│         결과를 블랙보드 {path} 에 저장                                │
│                    │                                                 │
│                    ▼                                                 │
│   ② SmoothPath      ──▶ smoother_server (ConstrainedSmoother)        │
│         {path} 를 읽어서 평활화 → 같은 {path} 에 덮어씀               │
│         (실패해도 주행 계속 — 아래 4장 참고)                          │
│                    │                                                 │
│                    ▼                                                 │
│   ③ FollowPath      ──▶ controller_server (MPPI)                     │
│         {path} 를 따라가는 /cmd_vel 을 20 Hz 로 발행                  │
└──────────────────────────────────────────────────────────────────────┘
        │
        ▼
   velocity_smoother  (가속 한계 적용)
        │
        ▼
   command_manager  (drive_mode 해석 + 안전 게이팅)
        │
        ▼
   mcu_bridge → STM32 (4WIS 역기구학)
```

①은 **1 Hz로 재계획**되고(BT의 `RateController hz="1.0"`), ③은 **20 Hz로 연속 실행**됩니다.

---

## 4. Behavior Tree — 가장 놓치기 쉬운 부분

**이번 작업에서 제일 중요한 함정입니다.**

`nav2.yaml`에 `smoother_server`를 설정하고 노드가 정상적으로 떠도, **Nav2 기본 BT에는
`SmoothPath` 노드가 없어서 평활화가 전혀 일어나지 않습니다.** 로그도 에러도 안 납니다.
그냥 조용히 아무 일도 안 일어납니다.

그래서 커스텀 BT 두 개를 새로 만들었습니다:

- `behavior_trees/navigate_to_pose_w_smoothing.xml` (단일 목표)
- `behavior_trees/navigate_through_poses_w_smoothing.xml` (경유지 주행)

Nav2 기본 트리에서 딱 이 부분만 추가한 것입니다:

```xml
<RecoveryNode number_of_retries="1" name="ComputePathToPose">
  <Sequence name="PlanAndSmooth">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>

    <!-- ↓↓↓ 여기가 새로 추가된 부분 ↓↓↓ -->
    <Fallback name="SmoothIfPossible">
      <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"
                  smoother_id="SmoothPath"
                  max_smoothing_duration="0.3"
                  check_for_collisions="true"/>
      <AlwaysSuccess/>
    </Fallback>
    <!-- ↑↑↑ 여기까지 ↑↑↑ -->

  </Sequence>
  <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
</RecoveryNode>
```

**`Fallback` + `AlwaysSuccess`로 감싼 이유**:
평활화는 어디까지나 "있으면 좋은" 단계입니다. 실패했다고 주행 전체를 멈추고 복구행동
(제자리 회전, 후진…)으로 빠지면 곤란합니다. 이렇게 감싸두면:

- 평활화 성공 → `{path}`가 매끄러운 경로로 갱신됨
- 평활화 실패 → `{path}`가 **갱신되지 않으므로 planner 원본 경로 그대로** 주행 계속

`check_for_collisions="true"`라서 평활화 결과가 장애물을 스치면 실패로 처리되고,
자동으로 안전한 원본 경로로 되돌아갑니다.

### BT 파일 경로는 어떻게 전달되나

`bt_navigator`는 BT xml의 **절대경로**를 파라미터로 받습니다. 그런데 YAML 파일에는
"패키지 share 디렉토리"를 표현할 방법이 없습니다. 그래서:

1. `nav2.yaml`에 **플레이스홀더**를 넣어둡니다 (키가 존재해야 치환이 됨):
   ```yaml
   bt_navigator:
     ros__parameters:
       default_nav_to_pose_bt_xml: "OVERRIDDEN_BY_LAUNCH"
       default_nav_through_poses_bt_xml: "OVERRIDDEN_BY_LAUNCH"
   ```
2. `navigation.launch.py`가 `RewrittenYaml`로 **실행 시점에 절대경로를 주입**합니다:
   ```python
   configured_params = RewrittenYaml(
       source_file=params_file,
       root_key="",
       param_rewrites={
           "default_nav_to_pose_bt_xml": os.path.join(bt_dir, "navigate_to_pose_w_smoothing.xml"),
           "default_nav_through_poses_bt_xml": os.path.join(bt_dir, "navigate_through_poses_w_smoothing.xml"),
       },
       convert_types=True,
   )
   ```

> 💡 `nav2.yaml`의 `default_nav_*_bt_xml` 두 줄을 **지우면 안 됩니다.** `RewrittenYaml`은
> 이미 존재하는 키만 치환하므로, 줄이 없으면 조용히 기본 BT로 돌아가고 평활화가
> 사라집니다.

---

## 5. costmap 변경 — 이건 선택이 아니라 필수였습니다

### 5-1. inflation_radius: 0.45/0.55 → 1.00/1.10

`inflation_radius`는 "장애물 주변을 얼마나 부풀려서 비용을 매길지"입니다.

이 로봇의 **circumscribed radius는 0.888 m**입니다. 그런데 기존 설정은 0.45 / 0.55 m로,
**circumscribed보다 작았습니다.** 이게 왜 문제냐면:

MPPI의 `CostCritic`은 매 궤적 점마다 이렇게 동작합니다.

```
중심점 costmap 비용을 본다
  └─ circumscribed 임계값보다 낮다  → 안전 확정, 통과 (빠름)
  └─ circumscribed 임계값 이상      → 전체 footprint 충돌검사 수행 (정확)
```

이 최적화가 성립하려면 **"circumscribed 반경 밖은 절대 충돌하지 않는다"** 가 보장되어야
하고, 그러려면 팽창 반경이 circumscribed보다 커야 합니다. 작으면 임계값 자체가 무의미해져
**충돌 가능한 궤적을 안전하다고 판정할 수 있습니다.**

```yaml
local_costmap:
  inflation_layer:
    cost_scaling_factor: 3.0
    inflation_radius: 1.00      # > circumscribed 0.888 ✓

global_costmap:
  inflation_layer:
    cost_scaling_factor: 2.5
    inflation_radius: 1.10      # 전역은 조금 더 여유
```

**전역에서의 부수효과(의도한 것)**: SmacPlanner2D는 로봇을 **점**으로 보고
`cost >= 253`(inscribed 등급) 셀만 장애물로 취급합니다. 즉 **통과 가능 폭이 inscribed
radius 0.52 m 기준으로 결정**되고, 그보다 바깥의 팽창은 "가능하면 벽에서 떨어져 달리자"는
선호도로 작동합니다. ConstrainedSmoother의 `w_cost`도 같은 costmap을 씁니다.

### 5-2. local costmap: 5×5 → 8×8 m

MPPI는 최대 `prune_distance`(2.0 m) 앞까지 보면서 2초짜리 궤적을 굴립니다. 거기에
차체 길이 1.37 m가 더해집니다. 기존 5×5 m(중심에서 ±2.5 m)는 롤아웃이 costmap 경계
밖으로 나가 평가가 잘려버릴 수 있는 크기였습니다.

8×8 m(±4 m)로 늘렸습니다. 0.05 m 해상도 기준 160 × 160 셀이라 메모리/연산 부담은 미미합니다.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      rolling_window: true
      width: 8      # 5 → 8
      height: 8     # 5 → 8
      resolution: 0.05
```

---

## 6. 변경 파일 전체 목록

### 새로 만든 파일 (3개)

| 파일 | 내용 |
|---|---|
| `alm_navigation/behavior_trees/navigate_to_pose_w_smoothing.xml` | 단일 목표 BT. `ComputePathToPose → SmoothPath → FollowPath` |
| `alm_navigation/behavior_trees/navigate_through_poses_w_smoothing.xml` | 경유지 BT. 위와 동일 구조 |
| `docs/auto.md` | 이 문서 |

### 수정한 파일 (5개)

| 파일 | 변경 요약 |
|---|---|
| `alm_navigation/config/nav2.yaml` | planner/controller 플러그인 교체, `smoother_server` 신설, costmap 조정, BT 경로 플레이스홀더 추가 |
| `alm_navigation/launch/navigation.launch.py` | `RewrittenYaml`로 BT 절대경로 주입 |
| `alm_navigation/CMakeLists.txt` | `behavior_trees` 디렉토리 설치 추가 |
| `alm_navigation/package.xml` | `nav2_smac_planner`, `nav2_constrained_smoother`, `nav2_mppi_controller`, `nav2_smoother`, `nav2_common` 의존성 추가 |
| `README.md`, `docs/OPERATION_GUIDE.md` | 아키텍처 다이어그램·설치 패키지·운영 절차 갱신 |

### nav2.yaml 변경 상세

| 섹션 | 변경 |
|---|---|
| 파일 상단 주석 | 새 파이프라인 설명 + 목표 yaw 제약 경고 |
| `bt_navigator` | `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` 플레이스홀더 2줄 추가 |
| `controller_server` | `FollowPath`를 DWB → MPPI로 전면 교체, `yaw_goal_tolerance` 0.20 → 0.25 |
| `local_costmap` | 5×5 → 8×8, `inflation_radius` 0.45 → 1.00 |
| `global_costmap` | `inflation_radius` 0.55 → 1.10, `cost_scaling_factor` 3.0 → 2.5 |
| `planner_server` | NavFn → SmacPlanner2D, `expected_planner_frequency` 5.0 → 1.0 |
| `smoother_server` | **신설** (ConstrainedSmoother) |
| `lifecycle_manager_navigation` | `node_names`에 `smoother_server` 추가 |

> `amcl` 섹션은 손대지 않았습니다. 이 프로젝트는 FAST-LIO/SC 기반 측위를 쓰므로
> 실제로는 사용되지 않는 죽은 설정입니다.

---

## 7. 빌드 & 실행

### 필요한 apt 패키지

플러그인 3종은 별도 데비안 패키지입니다. `ros-humble-navigation2`만 깔면 없습니다.

```bash
sudo apt install \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-constrained-smoother \
  ros-hu게mble-nav2-mppi-controller
```

(설치 확인: `dpkg -l | grep -E "nav2-smac|nav2-mppi|nav2-constrained"`)

### 빌드

```bash
cd ~/ALM_sc-lio-sam/ALM_auto_ws
colcon build --packages-select alm_navigation
source install/setup.bash
```

> ⚠ `behavior_trees/`가 설치되어야 BT를 찾습니다. `CMakeLists.txt`에 이미 추가해뒀지만,
> BT를 수정했는데 반영이 안 되면 **`colcon build`를 다시 돌렸는지** 확인하세요.
> `src/`의 xml을 고쳐도 `install/`에 복사되기 전엔 적용되지 않습니다.

### 실행

기존과 동일합니다. launch 인자는 바뀌지 않았습니다.

```bash
ros2 launch alm_navigation navigation.launch.py \
  map:=<2D map.yaml> \
  map_pcd:=<3D map.pcd>
```

---

## 8. 제대로 붙었는지 확인하기

세 플러그인이 실제로 로드됐는지 확인하는 게 가장 확실합니다.

```bash
ros2 param get /planner_server GridBased.plugin
#   → String value is: nav2_smac_planner/SmacPlanner2D

ros2 param get /smoother_server SmoothPath.plugin
#   → String value is: nav2_constrained_smoother/ConstrainedSmoother

ros2 param get /controller_server FollowPath.plugin
#   → String value is: nav2_mppi_controller::MPPIController

# BT가 커스텀 트리를 보고 있는지 (이게 "OVERRIDDEN_BY_LAUNCH"면 주입 실패!)
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
#   → .../install/alm_navigation/share/alm_navigation/behavior_trees/navigate_to_pose_w_smoothing.xml

# smoother_server 가 lifecycle active 인지
ros2 lifecycle get /smoother_server
#   → active [3]
```

**평활화가 실제로 도는지** 보려면 목표를 찍고 smoother 액션이 호출되는지 확인합니다.

```bash
ros2 topic echo /plan --once          # 경로 확인
ros2 node info /smoother_server       # smooth_path 액션 서버 존재 확인
```

**MPPI 궤적을 눈으로 보려면** (튜닝할 때만, CPU 부하 증가):

```bash
ros2 param set /controller_server FollowPath.visualize true
# RViz에서 /trajectories (MarkerArray), /optimal_trajectory (Path) 구독
```

---

## 9. 알려진 제약 — 반드시 읽어주세요

### 9-1. 목표 yaw가 무시됩니다

**SmacPlanner2D는 2D 격자 플래너라 경로에 자세(yaw) 정보가 없습니다.** 만들어낸 경로의
모든 pose는 orientation이 단위 사원수(yaw=0)입니다.

Nav2의 goal checker는 **경로의 마지막 pose**를 목표로 삼기 때문에, 그대로 두면
"항상 yaw=0으로 정렬하고 끝내라"는 뜻이 되어버립니다. 그래서
`use_final_approach_orientation: true`를 켜서 **마지막 pose만 진행방향으로 채웠습니다.**

결과:

- ✅ 도착 후 불필요한 제자리 회전이 없습니다 (자연스럽게 진입 방향 그대로 정지)
- ❌ **RViz에서 찍은 목표 화살표 방향(yaw)은 무시됩니다**

이 하나의 사실에서 아래 설정들이 전부 파생됩니다:

| 설정 | 값 | 이유 |
|---|---|---|
| `ConstrainedSmoother.reversing_enabled` | `false` | 자세가 없으니 후진 구간(cusp)을 판정할 수 없음 |
| `ConstrainedSmoother.keep_start_orientation` | `false` | 시작 자세가 무의미한 값이라 고정하면 경로가 뒤틀림 |
| `ConstrainedSmoother.keep_goal_orientation` | `true` | 마지막 pose만은 planner가 유효값으로 채워둠 |
| `PathAlignCritic.use_path_orientations` | `false` | 경로의 자세값을 믿으면 안 됨 |
| `general_goal_checker.yaw_goal_tolerance` | `0.25` | 진입 방향 기준이라 조금 넉넉하게 |

**목표 자세까지 맞춰야 한다면** `nav2.yaml`의 `GridBased.plugin`을
`nav2_smac_planner/SmacPlannerHybrid`(Hybrid-A*)로 바꾸세요. Hybrid-A*는 자세를 포함해
탐색하므로 목표 yaw를 지킵니다. 대신 `minimum_turning_radius`, `motion_model_for_search`
등을 추가로 설정해야 하고 계획 시간이 훨씬 오래 걸립니다.

### 9-2. 좁은 통로에서 기하학적으로 불가능한 경로가 나올 수 있습니다

SmacPlanner2D는 **로봇을 점으로 보고 회전 기하를 전혀 모릅니다.** 통과 가능 폭은
inscribed radius(0.52 m) 기준으로만 판정되므로, 폭 1.1 m 통로에서 직각으로 꺾는 경로가
나올 수 있습니다. 하지만 이 로봇은 **전장이 1.37 m**라 그런 회전이 실제로는 불가능합니다.

완화 수단은 있지만 보장은 아닙니다:

- `ConstrainedSmoother.minimum_turning_radius`가 급커브를 둥글게 펴줌
- MPPI `CostCritic.consider_footprint: true`가 실제 차체로 충돌을 걸러냄
- `command_manager`의 spin 모드가 제자리 회전으로 각을 잡음

좁은 실내를 자주 다닌다면 **Hybrid-A*로 가는 게 근본 해결책**입니다.

### 9-3. MPPI는 CPU를 많이 씁니다

매 주기 1600개 궤적 × 40 스텝을 계산합니다. Jetson에서 `controller_server`가 20 Hz를
못 채우면 (`/cmd_vel` 발행 주기가 떨어지거나 "controller missed rate" 경고) 이 순서로
낮추세요:

1. `FollowPath.batch_size` 1600 → 1000
2. `FollowPath.time_steps` 40 → 30
3. `CostCritic.trajectory_point_step` 2 → 3

반대로 여유가 있으면 `batch_size`를 올리면 궤적 품질이 좋아집니다.

> `model_dt`(0.05)는 `controller_frequency`(20 Hz)의 역수와 **맞춰야 합니다.**
> `controller_frequency`를 바꾸면 `model_dt`도 같이 바꾸세요.

### 9-4. `minimum_turning_radius: 0.60`은 아직 검증 전입니다

휠베이스 0.9116 m와 조향 한계 45° 가정에서 `R ≈ 휠베이스 / (2·tanδ)`로 추정한 값입니다.
실제 4WIS 조향각 스펙(toe-in 90° / toe-out 45°)으로 재검증이 필요해 yaml에 `##TODO##`로
표시해뒀습니다.

- 값을 **낮추면** 더 급한 회전을 허용 (경로가 각져짐)
- 값을 **올리면** 더 완만한 곡선만 허용 (경로가 크게 돌아감)

---

## 10. 튜닝 가이드 — 증상별 대처

| 증상 | 만져볼 파라미터 | 방향 |
|---|---|---|
| 벽에 너무 붙어서 지나감 | `GridBased.cost_travel_multiplier` | ↑ (2.0 → 3.0) |
| " | `SmoothPath.w_cost` | ↑ (0.020 → 0.05) |
| " | `inflation_radius` | ↑ |
| 너무 크게 돌아감 | `GridBased.cost_travel_multiplier` | ↓ |
| " | `inflation_layer.cost_scaling_factor` | ↑ (비용이 빨리 감쇠) |
| 경로가 여전히 각짐 | `SmoothPath.w_smooth` | ↑ (15000 → 30000) |
| " | `SmoothPath.path_downsampling_factor` | ↓ (3 → 2, 더 촘촘히 최적화) |
| 평활화된 경로가 원경로를 너무 벗어남 | `SmoothPath.w_dist` | ↑ (0.0 → 1.0) |
| 주행이 흔들림/사행 | `FollowPath.wz_std` | ↓ (0.30 → 0.20) |
| " | `PathAlignCritic.cost_weight` | ↑ (14 → 20) |
| 경로를 잘 못 따라감 (질러감) | `PathFollowCritic.cost_weight` | ↑ |
| " | `PathAlignCritic.cost_weight` | ↑ |
| 자꾸 후진하려 함 | `PreferForwardCritic.cost_weight` | ↑ (5.0 → 10.0) |
| " | `FollowPath.vx_min` | 0.0 으로 (후진 완전 금지) |
| 목표에서 멈칫거림 | `general_goal_checker.xy_goal_tolerance` | ↑ |
| " | `GoalCritic.cost_weight` | ↑ |
| 제어 주기 미달 (CPU) | [9-3](#9-3-mppi는-cpu를-많이-씁니다) 참고 | |
| 재계획이 너무 느림 | `GridBased.downsample_costmap: true` | 격자 축소로 가속 |

---

## 11. 되돌리는 방법

문제가 생겨 예전 스택으로 돌아가야 한다면, `nav2.yaml`에서 두 곳만 고치면 됩니다.
(BT는 자동으로 무시되지 않으므로 `bt_navigator` 설정도 함께 되돌려야 합니다.)

```yaml
# 1) planner
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

# 2) controller — FollowPath 블록 전체를 dwb_core::DWBLocalPlanner 설정으로 교체
#    (git 히스토리에서 이전 버전 참조)

# 3) BT — 두 줄을 지우면 Nav2 기본 트리로 복귀 (평활화 비활성)
bt_navigator:
  ros__parameters:
    # default_nav_to_pose_bt_xml: "OVERRIDDEN_BY_LAUNCH"
    # default_nav_through_poses_bt_xml: "OVERRIDDEN_BY_LAUNCH"
```

`git diff`로 이번 변경 전체를 확인할 수 있습니다:

```bash
git log --oneline -- ALM_auto_ws/src/alm_navigation/config/nav2.yaml
git diff <이전커밋> -- ALM_auto_ws/src/alm_navigation/
```

---

## 12. 참고 링크

- [Nav2 Smac Planner 문서](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)
- [Nav2 Constrained Smoother 문서](https://docs.nav2.org/configuration/packages/configuring-constrained-smoother.html)
- [Nav2 MPPI Controller 문서](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Nav2 Behavior Tree 노드 목록](https://docs.nav2.org/behavior_trees/index.html)
- 설치된 기본 BT 참고본: `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/`
