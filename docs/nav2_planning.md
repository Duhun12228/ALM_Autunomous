# Nav2 경로계획 — Hybrid-A* + ConstrainedSmoother + MPPI

```
목표(RViz/WebUI)
   │
   ▼
ComputePathToPose ── SmacPlannerHybrid ──▶ /plan  (R_min 을 지키는 경로, 자세 포함)
   │
   ▼
SmoothPath ──────── ConstrainedSmoother ─▶ /plan  (프리미티브 이음매 마감)
   │
   ▼
FollowPath ──────── MPPIController ──────▶ /cmd_vel
   │
   ▼
cmd_arbiter ─/cmd_vel_mux─▶ command_manager ─▶ mcu_bridge ─UART─▶ STM32
                            (조향각 한계 최종 보장)
```

연결은 `alm_navigation/behavior_trees/navigate_*_w_smoothing.xml` 이 담당합니다.
Nav2 기본 BT 에는 `SmoothPath` 노드가 없어서 커스텀 트리가 필요하고,
`navigation.launch.py` 가 `RewrittenYaml` 로 설치된 BT 의 절대경로를
`nav2.yaml` 의 `default_nav_*_bt_xml` 에 주입합니다.

---

## 1. 왜 Hybrid-A* 인가

이 플랫폼은 **normal 모드에서 자동차형**입니다. 최소 선회반경 아래로는 못 돕니다.

`SmacPlanner2D`(격자 A*)는 로봇을 **점**으로 보므로 90° 꺾인 경로를 태연히 냅니다.
그 경로를 스무더가 사후에 펴야 하는데, 스무더는 형상을 *완화*할 뿐 실현가능성을
*보장*하지 못합니다. 결과적으로 계획과 실행이 갈라집니다.

`SmacPlannerHybrid`(Hybrid-A*)는 상태공간을 `(x, y, yaw)` 로 놓고 **R_min 을
만족하는 모션 프리미티브만** 이어붙여 탐색하므로, 나오는 경로가 처음부터
실현가능합니다. 부수 효과가 셋 더 있습니다.

| | SmacPlanner2D | SmacPlannerHybrid |
|---|---|---|
| 경로의 곡률 | 제약 없음 (사후 평활화 의존) | **R_min 보장** |
| 경로 pose 의 자세(yaw) | 전부 0 (마지막만 진행방향) | **전 구간 유효** |
| 목표 자세 | 무시 | **지킴** |
| MPPI `use_path_orientations` | 못 켬 | **켬** (추종 품질 ↑) |
| 스무더의 역할 | 구제 (실패하면 곤란) | 마감 (실패해도 안전) |
| 탐색 비용 | 낮음 | 높음 (yaw 축 추가) |

`use_path_orientations: true` 를 켤 수 있게 된 것이 실질적인 이득입니다.
2D 경로에서는 yaw 가 전부 0이라 이 옵션을 켜면 MPPI 가 엉뚱한 방향으로
정렬하려 들어 반드시 false 여야 했습니다.

### ★ 반드시 알아야 할 성질: 경로는 **현재 헤딩에 접해서** 출발한다

상태공간이 `(x, y, yaw)` 라는 말은 **시작 상태의 yaw 도 고정 입력**이라는 뜻입니다.
planner_server 는 `map->base_link` TF 에서 위치와 함께 **orientation 도** 가져가고
(BT 의 `ComputePathToPose` 에 `start` 포트가 없으므로), 확장은 R_min 원호뿐입니다.

따라서 **경로는 언제나 로봇이 지금 향한 방향으로 출발합니다.** 벽을 보고 서 있으면
전역경로는 "벽 쪽으로 나가서 R_min 으로 크게 감아 돌아라" 라고 말하고, 그건 경로
입장에서 **정답**입니다. 목표가 뒤에 있어도 제자리 회전을 표현할 수 없으니 지름
2·R_min = 3.29 m 의 U턴이나 Reeds-Shepp 스위치백밖에 없습니다.

이 성질에는 제어단에서 중요한 따름정리가 있습니다. 출발 시점에 '경로 헤딩오차'
(= lookahead 지점 pose.yaw − 내 yaw)가 낼 수 있는 최대값이 호각으로 묶입니다:

```
|경로 헤딩오차|_max = lookahead / R_min
    lookahead 1.00 m ->  34.9°
    lookahead 1.72 m ->  60.0°
    lookahead 3.00 m -> 104.6°
```

##함정## `align_enter_deg`(60°)를 `align_lookahead_m`(1.0)로 재면 **출발 시점에는
수학적으로 절대 안 걸립니다.** 실제로 그 상태로 한동안 굴렸습니다 —
경위와 수정은 `docs/control_pipeline.md` §6.11, 검사는 `nav2_kinematic_check.py`.

**정리하면**: 전역 플래너에게 "출발 헤딩을 고쳐 달라" 고 요구할 수 없습니다.
플래너는 주어진 헤딩에서 최선을 낼 뿐입니다. 헤딩을 고치는 것은 제어단(ALIGN)의
일이고, 그 판단은 **경로가 아니라 목표 방위각**을 봐야 합니다.

### 이 맵에서 쓸 수 있는가 (alm_lab, 50.7 × 33.6 m)

Hybrid-A* 의 위험은 "좁은 곳에서 경로를 못 찾는다"입니다. 실제 맵으로 확인했습니다.

| 영역 | 넓이 | 비율 |
|---|---|---|
| 로봇 중심이 놓일 수 있는 곳 (여유 ≥ inscribed 0.53 m) | 1429.7 m² | 84.1% |
| 회전 여유가 있는 곳 (≥ circumscribed 0.894 m) | 1297.9 m² | 76.3% |
| R_min 선회가 가능한 곳 (≥ 1.643 m) | 1079.5 m² | 63.5% |
| U턴이 가능한 곳 (≥ R_min + inscribed) | 956.9 m² | 56.3% |

**주행가능 영역의 99.9% 가 하나로 이어져 있습니다**(1427.9 / 1429.7 m²).
위상적으로 막힐 일은 없습니다.

다만 *제자리에서 방향을 바꿀 수 있는* 영역은 7조각으로 나뉩니다. 그래서
전진 전용(`DUBIN`)이 아니라 **후진을 허용하는 `REEDS_SHEPP`** 을 씁니다.
평소에 후진이 나오지 않도록 `reverse_penalty: 3.0` 으로 비싸게 매겼습니다 —
이 값은 실제 속도비(전진 0.45 / 후진 0.15 = 3.0)와 같아서, "후진은 3배 느리니
3배 비싸다"가 그대로 비용함수에 반영됩니다.

---

## 2. 숫자는 전부 워크스페이스에서 유도했습니다

`nav2.yaml` 에 손으로 넣은 상수는 없습니다. 출처는 둘뿐입니다.

- `alm_base_control/config/base_control.yaml` — STM32 `ALM07.slx` 의 CONS 벡터
- `alm_description/urdf/alm_robot.urdf.xacro` — CAD 실측 지오메트리

유도는 `alm_base_control/scripts/fourwis_encode.py` 가 합니다.

```
기구 상수:  B = 1.000 m (CONS(2))   T = 0.919 m (CONS(3))
            rws_ratio = 0.5 (CONS(1), 후륜 50% 역조향)   max_steer = 30°

  ↓ fourwis_encode.min_turn_radius()

R_min = 1.643 m           →  planner/smoother 의 minimum_turning_radius
normal 최대 요레이트       →  |wz| ≤ |vx| / R_min  = 0.274 rad/s @ vx=0.45
analytic_expansion_max_length = 2 × R_min = 3.29  →  3.5 (Nav2 권장)
```

> ⚠ **후륜 역조향이 반영된 값입니다.** 후륜 고정(rws=0)으로 계산하면 2.079 m 가
> 나오는데, 그건 이 플랫폼의 실제 기구가 아닙니다. 손계산으로 검증할 때
> 주의하세요.

footprint 는 URDF 에서:

```
앞끝 x = front_x 0.6106 + wheel_r 0.103              = 0.7136
뒤끝 x = min(rear_x -0.301 - 0.103, -body_len/2)     = -0.600
최대 |y| = max(half_track 0.5 + wheel_w/2, body_w/2) = 0.5244
  → 여유 포함 채택  x[-0.65, +0.72],  y[±0.53]
     inscribed 0.530 m   circumscribed 0.894 m
```

`inflation_radius` 는 반드시 circumscribed(0.894) 보다 커야 costmap 충돌평가와
MPPI `CostCritic` 의 임계값이 성립합니다 → local 1.00, global 1.20.

### 정합성 검사

기구 상수를 바꾸면 위 유도값이 전부 따라 바뀌는데, `nav2.yaml` 은 그걸 알 방법이
없습니다. 어긋나도 어느 로그에도 안 찍힙니다. 그래서 검사기를 두었습니다.

```bash
ros2 run alm_navigation nav2_kinematic_check.py
```

`base_control.yaml` 을 읽어 `fourwis_encode` 로 유도값을 계산하고 `nav2.yaml` 과
대조합니다. 읽기 전용이며, 고쳐주지 않고 어디가 어긋났는지만 알려줍니다
(종료코드 0 = 일치, 1 = 어긋남). 빌드 전에도 소스 트리를 읽어 동작합니다.

---

## 3. 조향 제한 — 계획과 실행을 잇는 마지막 고리

Hybrid-A* 가 R_min 을 지키는 경로를 내도, MPPI 는 추종 오차를 메우느라
순간적으로 그보다 급한 곡률을 명령할 수 있습니다. 그대로 두면 STM32 의
조향각만 30° 에서 포화되고, `/mcu/command` 의 `cmd_vel` 은 여전히 원래 값이라
**두 필드가 서로 다른 이야기를 합니다.**

`command_manager` 가 마지막에 접습니다.

```
|wz| ≤ |vx| / R_min          (R_min 은 fourwis_encode 가 런타임 계산)
|vx| < steer_limit_min_vx    →  wz = 0   (normal 모드는 제자리 회전 불가)
```

가속 램프에서도 비가 깨지지 않도록 **목표 twist 와 가속제한 통과 후 값 양쪽에**
적용합니다. wz 의 가속한계(1.5 rad/s²)가 vx(1.0 m/s²)보다 커서 출발 직후
wz/vx 비가 순간적으로 한계를 넘기 때문입니다. Nav2 쪽에서도
`velocity_smoother.scale_velocities: True` 로 비를 유지시켜 클램프가 개입할
일 자체를 줄였습니다.

### 실측 검증

```
기동:  R_min 1.643 m,  vx=0.45 에서 최대 wz=0.274 rad/s
       조향 제한 ON: |wz| <= |vx|/1.643, vx<0.03 이면 wz=0

요청 vx=0.30 wz=0.30 (R=1.00 m)  ->  wz 0.300 -> 0.183   (= 0.30/1.6425 ✓)
요청 vx=0.10 wz=0.30 (R=0.33 m)  ->  wz 0.300 -> 0.061   (= 0.10/1.6425 ✓)
요청 vx=0.01 wz=0.20             ->  wz 0.200 -> 0       (vx < 0.03 ✓)
모드 전환 횟수: 1 (기동 시 1회, 왕복 없음)
```

### 왜 '반경이 작으면 spin 으로' 라우팅하지 않는가

sc-lio-sam 브랜치의 `auto_spin_route_radius` 처럼 "요구 선회반경 < R_min 이면
제자리 회전으로 넘긴다"를 넣었다가 **뺐습니다.**

`command_manager` 의 auto 상태머신은 spin 을 *제자리 회전*(선속도 ≈ 0)으로
정의합니다. 반경 조건은 전진 중에도 참이 될 수 있어 두 정의가 충돌합니다.
spin 으로 들어가면 `vx` 가 0 으로 눌리고, 다음 틱에 다시 판정하면서
모드가 왕복할 여지가 생깁니다.

Hybrid-A* 를 쓰면 애초에 경로가 R_min 을 지키므로, 한계를 넘는 요청은
**드물고 일시적인 추종 오차**입니다. 그런 전이 상태를 "정지 → 회전 → 재출발"
이라는 큰 기동으로 처리하는 것은 과합니다. 클램프로 조금 크게 도는 편이 낫고,
그 오차는 MPPI 가 피드백으로 메웁니다 — 폐루프 제어기가 원래 하는 일입니다.

진짜 제자리 회전이 필요하면 **ALIGN 이 명시적으로 판단해서** 겁니다
(`docs/control_pipeline.md` §6.8 · §6.11). 이게 이 문단이 쓰인 뒤에 바뀐 부분입니다.

> **갱신 (2026-08-23/25)**: MPPI 를 Ackermann 으로 바꾸면서 이 절의 전제 하나가
> 바뀌었습니다. MPPI 의 실효 wz 상한이 `|vx|/R_min = 0.122` 이고
> `auto_spin_angular_threshold` 가 0.20 이므로, **MPPI 가 낸 명령으로 spin 라우팅이
> 걸리는 일은 이제 없습니다.** spin 을 거는 주체는 둘뿐입니다:
>
> * **ALIGN** — `/plan` 헤딩오차(주행 중) 또는 목표 방위각(출발 구간)을 보고 선제 판단
> * **BT 리커버리 Spin** — `behavior_server.max_rotational_vel` 0.25 로 문턱을 넘김
>
> 위 '사각지대' 경고도 그래서 성격이 바뀌었습니다. MPPI 가 스스로 wz 를 키워
> 빠져나오는 경로는 이제 없고, 대신 ALIGN 과 리커버리가 담당합니다.

---

## 4. 튜닝 손잡이

### Orin Nano 에서 느릴 때 (순서대로)

| 증상 | 손잡이 | 비고 |
|---|---|---|
| 기동이 오래 걸림 | `lookup_table_size` ↓ | **실측**: 20.0 → 8.19 s, 10.0 → 2.40 s (개발 PC). 현재 10.0 |
| 계획이 `max_planning_time`(2 s)을 넘김 | `downsample_costmap: true` + `downsampling_factor: 2` | 0.05 → 0.10 m. 자유공간 84% 가 여유 0.53 m 이상이라 통로가 막히지 않음 |
| 〃 (그래도 느림) | `angle_quantization_bins` 72 → 64 | 5.0° → 5.6° |
| `controller_server` CPU 포화 | `batch_size` 1600 → 1000, 그 다음 `time_steps` 56 → 40 | 지평선이 짧아지므로 코너 진입이 늦어짐 |

`cache_obstacle_heuristic` 은 기본 `false` 입니다. 전역 costmap 에 동적
`obstacle_layer` 가 붙어 있어 캐시가 낡으면 탐색이 엉뚱하게 퍼집니다.
성능이 급하면 켜되, 정적 환경에서만 쓰세요.

### 주행 품질

| 증상 | 손잡이 |
|---|---|
| 경로가 벽에 너무 붙음 | `cost_penalty` ↑, `global_costmap.inflation_radius` ↑ |
| 경로가 과하게 돌아감 | `cost_penalty` ↓, `cost_scaling_factor` ↑ |
| 평활화 후에도 급회전 | `smoother.w_curve` ↑ (현재 200, nav2 기본 30은 `w_smooth` 15000 대비 너무 작아 무시됨) |
| 목표 근처에서 맴돎 | `yaw_goal_tolerance` ↑, `GoalAngleCritic.cost_weight` ↓ |
| 좁은 곳에서 계획 실패 | `tolerance` ↑, `analytic_expansion_ratio` ↑, 마지막 수단으로 `SmacPlanner2D` 로 교체 |
| 후진이 잦음 | `reverse_penalty` ↑ 또는 `motion_model_for_search: "DUBIN"` |

---

## 5. 검증 상태

| 항목 | 상태 |
|---|---|
| `planner_server` (SmacPlannerHybrid, Reeds-Shepp) 설정 로드 | ✅ 확인 |
| `smoother_server` (ConstrainedSmoother) 설정 로드 | ✅ 확인 |
| `controller_server` (MPPI, critic 8종) 설정 로드 | ✅ 확인 |
| 커스텀 BT 2종 파싱 (`bt_navigator` configure) | ✅ 확인, 에러 0 |
| `nav2_kinematic_check.py` 정합성 | ✅ 전 항목 일치 |
| `command_manager` 조향 클램프 실측 | ✅ 유도값과 일치 |
| **실제 경로 생성 / 주행** | ❌ **미검증** — TF·맵·측위가 붙은 전체 스택 필요 |

> ⚠ 위 검증은 전부 "설정이 받아들여지고 플러그인이 뜬다"까지입니다.
> **경로가 실제로 나오는지, 로봇이 그 경로를 따라가는지는 아직 확인하지
> 않았습니다.** `webui_dev.launch.py` + `pcd_replay` 로 하드웨어 없이
> 경로 생성까지는 볼 수 있습니다.

## 관련 파일

- `alm_navigation/config/nav2.yaml` — 파라미터 (유도 과정을 주석에 기록)
- `alm_navigation/behavior_trees/navigate_*_w_smoothing.xml` — SmoothPath 를 끼운 BT
- `alm_navigation/launch/navigation.launch.py` — BT 경로 주입 (`RewrittenYaml`)
- `alm_navigation/scripts/nav2_kinematic_check.py` — 정합성 검사
- `alm_base_control/scripts/command_manager.py` — 조향각 한계 최종 보장
- `alm_base_control/scripts/fourwis_encode.py` — R_min 등 기구학 계산 (단일 진실 공급원)
- `docs/control_arbitration.md` — `/cmd_vel` 이후의 경로
