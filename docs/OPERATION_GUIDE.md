# ALM 자율주행 운영 가이드

> **이 문서는 `dev/sc-lio-sam` 브랜치(= 방식 C) 기준입니다.**
> 매핑은 **SC-LIO-SAM**(LIO-SAM + Scan Context 루프클로저), 초기위치는
> **Scan Context 자동특정**을 사용합니다. 브랜치별 차이는 [README](../README.md)의
> 방식 비교표를 참고하세요.

기존 2D `slam_toolbox + AMCL + EKF` 흐름은 사용하지 않습니다.

## 핵심 실행 경로


```text
Livox MID-360 UDP 직접 파싱
  -> /livox/lidar, /livox/imu, /scan
SC-LIO-SAM 매핑 (RS + Scan Context 루프클로저, GTSAM 팩터그래프)
  -> GlobalMap.pcd
pcd2pgm
  -> alm_map.pgm/yaml            (Nav2 costmap 용)
sc_build_db
  -> sc_db.npz                   (Scan Context 초기위치 DB)
Scan Context 자동 초기화
  -> /initialpose -> ICP -> map->odom
FAST-LIO 측위 추적
  -> odom->base_link, /Odometry
Nav2  SmacPlanner2D -> ConstrainedSmoother -> MPPI
  -> /cmd_vel
command_manager + mcu_bridge
  -> /mcu/command -> STM32
```

**Scan Context는 두 군데에서 서로 다른 일을 합니다.**

| | 어디서 | 무슨 일 | 구현 |
|---|---|---|---|
| 매핑 중 | `mapOptimization` | 재방문 인식 → **루프클로저** (드리프트 보정) | C++ [`Scancontext.cpp`](../ALM_auto_ws/src/thirdparty/SC-LIO-SAM/src/Scancontext.cpp) |
| 측위 시작 | `sc_localizer` | **초기위치 자동특정** (RViz 수동 지정 대체) | Python [`scan_context.py`](../ALM_auto_ws/src/alm_navigation/scripts/scan_context.py) |

---

## 0. 센서 / TF 기준

| 용도 | 입력 → 출력 | 담당 |
|---|---|---|
| 3D 점군 | `/livox/lidar` | `alm_sensors/scripts/livox_udp_pointcloud2.py` |
| 내장 IMU | `/livox/imu` | `alm_sensors/scripts/livox_udp_imu.py` |
| EKF용 IMU relay | `/imu/data` | `imu_relay.py` |
| 2D costmap용 scan | `/scan` | `pointcloud_to_scan.py` |
| **3D 매핑** | `/livox/lidar` + `/livox/imu` → PCD | **SC-LIO-SAM** (4노드 + IMU 필터) |
| **초기위치 자동특정** | `/livox/lidar` + `sc_db.npz` → `/initialpose` | **`sc_localizer.py`** |
| 재측위 정합 | prior PCD + 현재 scan → `map->odom` | `icp_node` + `transform_publisher` |
| 실시간 추적 | LiDAR/IMU → `odom->base_link`, `/Odometry` | FAST-LIO |
| 주행 명령 | `/cmd_vel` → `/mcu/command` | `command_manager` |

주행 모드에서는 측위 스택이 TF를 담당하므로 **EKF를 끕니다**.
매핑 모드에서는 `robot.launch.py` 기본값 때문에 EKF가 켜질 수 있지만,
맵 자체는 LiDAR+IMU 기반이며 엔코더는 맵 생성에 관여하지 않습니다.

### ⚠️ 시작 전 실측이 필요한 값

| 값 | 위치 | 상태 |
|---|---|---|
| 라이다 마운트 높이 | [`lidar.launch.py`](../ALM_auto_ws/src/alm_sensors/launch/lidar.launch.py) `lidar_z` | **`0.5` 는 임시값 (`##TODO## 실측`)** |
| 천장 높이 | `sc_build_db --z-max` 상한 결정용 | 미측정 |

Scan Context의 `z_min`/`z_max`는 **지면이 아니라 라이다 기준**입니다.
마운트 높이를 모르면 실제로 어느 높이대를 보고 있는지 알 수 없고,
천장이 밴드에 들어가면 **모든 칸이 천장 높이로 균일해져 장소 구분이 무너집니다.**

---

## 1. 하드웨어 기본 스택 확인

```bash
WS=~/ALM_Autunomous/ALM_auto_ws     # 브랜치 체크아웃 경로에 맞게
MAPS=$WS/src/alm_navigation/maps

cd $WS
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch alm_bringup robot.launch.py
```

다른 터미널에서 확인:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /scan
ros2 topic hz /mcu/state
ros2 topic hz /wheel_odom
ros2 run tf2_tools view_frames
```

`/scan`이 비어 있으면 LiDAR 장착 높이와 `pointcloud_to_scan`의
`min_height/max_height`를 조정합니다.

---

## 2. 3D 매핑 (SC-LIO-SAM)

```bash
# 터미널 1 — 센서
ros2 launch alm_sensors lidar.launch.py

# 터미널 2 — SC-LIO-SAM (4노드 + IMU 필터)
ros2 launch alm_navigation slam_sc.launch.py rviz:=true
```

로봇을 천천히 움직이며 공간 전체를 훑습니다.
**루프를 닫으면(같은 곳으로 되돌아오면)** 콘솔에 이런 로그가 뜹니다:

```text
SC loop found! between 412 and 87.
ICP fitness test passed (0.08 < 0.3). Add this SC loop.
```

이 로그가 뜨면 팩터그래프가 보정되어 누적 드리프트가 줄어듭니다.
**한 번도 안 뜬다면** 루프를 제대로 안 닫았거나 Scan Context 파라미터가
공간에 안 맞는 것입니다 ([`Scancontext.h`](../ALM_auto_ws/src/thirdparty/SC-LIO-SAM/include/lio_sam/Scancontext.h)의
`PC_MAX_RADIUS`, `PC_MAX_Z`, `SC_DIST_THRES` 확인).

맵 저장 (종료 전):

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.0, destination: ''}"
```

산출물:

```text
$MAPS/sc_lio_sam/GlobalMap.pcd      ← 이후 모든 단계의 기준 맵
$MAPS/sc_lio_sam/trajectory.pcd     ← 경사/장거리 맵의 2D 변환에 사용
```

> **이 브랜치의 맵 파일명은 `GlobalMap.pcd`입니다.** 이전 FAST-LIO 방식의
> `alm_3d_map.pcd`와 다릅니다. 이후 모든 명령에서 **경로를 명시**하세요.
> 자세한 이유는 [§8 알려진 이슈](#8-알려진-이슈와-주의사항) 참고.

PCD는 대용량 로컬 산출물이라 `.gitignore`에서 제외됩니다.

---

## 3. 3D PCD → Nav2용 2D 맵

```bash
ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAPS/sc_lio_sam/GlobalMap.pcd \
  --out $MAPS/alm_map \
  --resolution 0.05 \
  --z-min 0.3 --z-max 0.8
```

장거리·경사가 있는 맵은 전역 z 대신 **궤적 기준 상대 높이**를 씁니다:

```bash
ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAPS/sc_lio_sam/GlobalMap.pcd \
  --trajectory $MAPS/sc_lio_sam/trajectory.pcd \
  --out $MAPS/alm_map \
  --resolution 0.10 --z-min -1.45 --z-max 0.35 --min-points 2
```

출력: `alm_map.pgm`, `alm_map.yaml`

`pcd2pgm.py`가 찍는 z 분포를 보고 벽/장애물만 잡히도록 밴드를 조정합니다.

---

## 4. Scan Context DB 생성 (맵 갱신 때마다 1회, 오프라인)

초기위치 자동특정에 쓸 DB를 미리 구워둡니다. **로봇 없이 돌아갑니다.**

```bash
ros2 run alm_navigation sc_build_db.py \
  --pcd $MAPS/sc_lio_sam/GlobalMap.pcd \
  --out $MAPS/sc_db.npz \
  --step 0.5 \
  --selftest 20
```

### 출력 해석

```text
[sc_build_db] 격자 1240곳 중 키프레임 903개 (점부족 제외 210, 장애물내부 제외 127, ...)
[sc_build_db] 링별 평균 점유율 (안쪽→바깥, 링폭 0.50m):
  0.00 0.02 0.11 0.18 0.24 ... 0.03 0.01 0.00
```

**링별 평균 점유율이 가장 중요한 진단 지표입니다.** 이 20개 숫자가
**실제 스캔의 링 프로파일과 비슷해야** 매칭이 성립합니다.

DB는 맵(= 여러 위치에서 본 것의 합집합)에서 만들기 때문에, 보정 없이는
**벽 뒤 옆방까지 들어가서 바깥 링만 부풀어 오릅니다.** 실제 라이다는 그 자리에서
벽 너머를 못 보므로, 이 비대칭이 있으면 **매칭이 트인 곳으로 계통적으로 밀립니다**
(증상: *방향은 맞는데 위치만 구조물 반대쪽으로 수 m 이탈*).

그래서 DB 생성 시 맵 점군을 **"그 자리에서 실제로 보이는 것"으로 깎습니다**
([`simulate_scan`](../ALM_auto_ws/src/alm_navigation/scripts/sc_build_db.py)):

| 보정 | 옵션 | 기본 |
|---|---|---|
| 가려짐(벽 투과) 제거 — 방위·고도 깊이버퍼 | `--no-visibility` 로 끔 | **ON** |
| 수직 시야각 밖 제거 | `--fov-down 7 --fov-up 52` | MID-360 값 |

효과 비교 (합성 2방 맵):

| 지표 | 필터 OFF | 필터 ON |
|---|---:|---:|
| ring_key 거리 (1단계 후보추리기) | 0.333 | **0.041** |
| SC 거리 | 0.136 | **0.017** |
| 정답 vs 오답 점수 격차 | 4배 | **35배** |

### ⚠️ `--selftest` 결과를 신뢰 지표로 쓰지 마세요

selftest는 가상 스캔을 **DB와 똑같은 방식으로** 만듭니다. 양쪽이 같은 왜곡을
공유하므로 **20/20 통과해도 실전은 0/20일 수 있습니다.**
검증하는 것은 "격자 양자화와 회전 복원"뿐이고, 실제 라이다의 가려짐·시야각·희소성은
전혀 보지 않습니다.

### 주요 파라미터

| 옵션 | 기본 | 의미 |
|---|---:|---|
| `--step` | 0.75 m | DB 격자 간격. 작을수록 후보 위치 오차↓, 생성 시간↑ |
| `--max-radius` | 10.0 m | SC 반경. **공간 크기에 맞출 것** — 벽 밖 링은 낭비 |
| `--z-min` / `--z-max` | -0.3 / 1.0 | **라이다 기준** 높이 밴드. `z_max`는 반드시 천장 아래 |
| `--min-points` | 2000 | 키프레임 유효 최소 점 수 |

---

## 5. 측위만 검증

```bash
# 터미널 1 — 센서
ros2 launch alm_sensors lidar.launch.py

# 터미널 2 — 측위 (sc_localizer + icp + transform_publisher + fastlio)
ros2 launch alm_navigation localization.launch.py \
  map_pcd:=$MAPS/sc_lio_sam/GlobalMap.pcd \
  sc_db:=$MAPS/sc_db.npz
```

> **`map_pcd`와 `sc_db`는 반드시 같은 맵에서 나온 짝이어야 합니다.**
> 기본값이 이전 방식의 `alm_3d_map.pcd`를 가리키므로 **생략하면 안 됩니다.**

동작:

```text
COLLECT : /livox/lidar 10프레임 누적 (약 1초, 로봇 정지 상태여야 함)
MATCH   : SC 디스크립터 → DB 대조 → 상위 5개 후보
WAIT    : 후보를 /initialpose 로 발행 → ICP 수렴 대기 (후보당 12초)
성공    : /icp_result → transform_publisher → TF map->odom → sc_localizer 종료
```

로그 예:

```text
[sc_localizer] SC DB 로드: .../sc_db.npz (키프레임 903개, ring 20 x sector 60, r<10.0m)
[sc_localizer] [시도 1] 스캔 214033점 매칭 완료, 상위 후보: (12.5,3.0,87deg d=0.142), ...
[sc_localizer] 후보 1 발행: x=12.50 y=3.00 yaw=87deg (sc_dist=0.142) — ICP 수렴 12s 대기
[icp_node]     ICP fitness score: 0.081
[sc_localizer] ICP 수렴 — 측위 성공: x=12.43 y=3.11 z=0.02. sc_localizer 종료
```

**RViz에서 확인:**

```bash
ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=$MAPS/alm_map.yaml
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz
```

디버그 토픽:

| 토픽 | 내용 |
|---|---|
| `/sc_candidates` | SC 후보들 (PoseArray, map 프레임) |
| `/map` | `icp_node`가 읽은 prior map (다운샘플) |
| `/transformed_cloud` | 정렬된 현재 스캔 — **맵과 겹치는지 눈으로 확인** |

수동 초기화로 돌리려면:

```bash
ros2 launch alm_navigation localization.launch.py \
  map_pcd:=$MAPS/sc_lio_sam/GlobalMap.pcd auto_init:=false
```
→ RViz의 **2D Pose Estimate**로 직접 지정.

---

## 6. 자율주행

```bash
ros2 launch alm_bringup navigation.launch.py \
  map:=$MAPS/alm_map.yaml \
  map_pcd:=$MAPS/sc_lio_sam/GlobalMap.pcd
```

함께 올라오는 것:

- `robot.launch.py use_ekf:=false`
- `nav2_map_server`
- 측위 스택 (sc_localizer + ICP + FAST-LIO)
- Nav2 planner/smoother/controller/BT

**Nav2 Goal**을 지정하면 `/cmd_vel`이 생성되고, `command_manager`가 `auto` 모드에서
`normal/spin/crab` 중 실제 MCU에 보낼 모드를 선택합니다. crab은 기본 비활성입니다.

### 경로계획 파이프라인

| 단계 | 서버 | 플러그인 | 출력 |
|---|---|---|---|
| 전역계획 | `planner_server` | `nav2_smac_planner/SmacPlanner2D` | `/plan` |
| 평활화 | `smoother_server` | `nav2_constrained_smoother/ConstrainedSmoother` | 평활화된 `/plan` |
| 지역제어 | `controller_server` | `nav2_mppi_controller::MPPIController` | `/cmd_vel` |

세 단계를 잇는 것은 커스텀 BT
`alm_navigation/behavior_trees/navigate_to_pose_w_smoothing.xml` 입니다
(`ComputePathToPose` → `SmoothPath` → `FollowPath`). Nav2 기본 BT에는 `SmoothPath`가
없어서, 이 파일을 쓰지 않으면 `smoother_server`가 떠 있어도 호출되지 않습니다.
BT 절대경로는 `navigation.launch.py`가 `RewrittenYaml`로 `nav2.yaml`의
`default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`에 주입합니다.

확인/디버깅:

```bash
ros2 param get /planner_server GridBased.plugin        # SmacPlanner2D
ros2 param get /smoother_server SmoothPath.plugin      # ConstrainedSmoother
ros2 param get /controller_server FollowPath.plugin    # MPPIController
ros2 param get /bt_navigator default_nav_to_pose_bt_xml

# MPPI 롤아웃 시각화 (CPU 부하 증가, 튜닝할 때만)
ros2 param set /controller_server FollowPath.visualize true
# -> RViz 에서 /trajectories (MarkerArray), /optimal_trajectory (Path) 구독
```

**알려진 제약**

- `SmacPlanner2D`는 2D 격자 플래너라 경로에 자세(yaw)가 없습니다.
  `use_final_approach_orientation: true`로 마지막 pose만 진행방향으로 채우므로
  **RViz에서 찍은 목표 yaw는 무시**됩니다. 목표 자세까지 맞춰야 하면
  `nav2.yaml`의 `GridBased.plugin`을 `nav2_smac_planner/SmacPlannerHybrid`로
  바꾸고 `minimum_turning_radius` / `motion_model_for_search`를 설정하세요.
- 로봇이 1.37 × 1.04 m 로 커서 통과 가능 폭은 inscribed radius(0.52 m) 기준으로
  결정됩니다. `SmacPlanner2D`는 회전 기하를 모르므로, 좁은 통로에서 직각으로 꺾는
  경로가 나오면 `ConstrainedSmoother`의 `minimum_turning_radius`와
  MPPI 의 실제 추종 능력에 의존하게 됩니다.
- MPPI는 CPU를 많이 씁니다. Jetson에서 `controller_server`가 20 Hz를 못 채우면
  `FollowPath.batch_size`(1600) → 1000, `time_steps`(40) → 30 순으로 낮추세요.

---

## 7. 주행 모드

`/drive_mode`는 `std_msgs/String`입니다.

```bash
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

| 모드 | 동작 |
|---|---|
| `normal` | 전후진 + 회전 |
| `spin` | 제자리 회전 |
| `crab` | 측면 병진 (기본 자동 선택 비활성) |
| `auto` | `/cmd_vel`을 보고 normal/spin 자동 전환 |

실제 적용된 모드는 `/drive_mode/effective`에서 확인합니다.

---

## 8. 알려진 이슈와 주의사항

> 아래 항목은 **코드에서 확인된 사실**입니다. 실측 검증은
> [`SC_ICP_실험체크리스트.md`](SC_ICP_실험체크리스트.md) 참고.

### 8-1. `map_pcd` 기본값이 이 브랜치와 안 맞습니다

[`localization.launch.py`](../ALM_auto_ws/src/alm_navigation/launch/localization.launch.py)의
기본값은 `maps/alm_3d_map.pcd`(FAST-LIO 시절 이름)이고, 파일에 `##TODO##`가
그대로 남아 있습니다. 이 브랜치는 `GlobalMap.pcd`를 만듭니다.

**증상**: SC는 자기 DB 맵 기준으로 후보를 잘 내는데, ICP는 다른 맵을 읽어서
정합이 전혀 안 됨. `fitness`가 크게 나오고 무엇을 바꿔도 안 변함.

**확인**:
```bash
ros2 param get /icp_node map_path      # DB 만들 때 쓴 pcd 와 같은가?
```

### 8-2. `max_correspondence_distance = 0.1 m`가 SC 후보에 비해 너무 좁습니다

이 값은 ICP가 "짝을 지을지" 판단하는 거리입니다. 원래 **RViz로 사람이 찍어준
초기값(오차 ~10 cm)** 을 전제로 튜닝된 값인데, Scan Context가 주는 후보는 훨씬 거칩니다.

| 오차원 | 크기 |
|---|---|
| DB 격자 `step=0.75m` | ±0.53 m |
| yaw 양자화 6° | 10 m 지점에서 약 1 m |
| **맵 voxel 0.5 m 이산화** | **정렬이 완벽해도 최근접점이 ~0.25 m** |

세 번째가 특히 중요합니다 — **정답 위치에 정확히 놓아도 짝의 상당수가 버려집니다.**

**권장**: `max_correspondence_distance ≥ max(맵 voxel × 3, 초기 오차)` → 현재 조건에서 **1.5 m**.
정석은 Coarse(2.0 m) → Fine(0.6 m, voxel도 함께 축소) 2단계.

### 8-3. `fitness_score`가 맵 밖 점 때문에 부풀려집니다

[`icp_node.cpp`](../ALM_auto_ws/src/thirdparty/Fast-LIO2-Localization/icp_relocalization/src/icp_node.cpp)가
`getFitnessScore()`를 **인자 없이** 호출합니다 → PCL이 컷오프를 무한대로 잡아
**맵에 대응점이 아예 없는 점까지 벌점으로 셉니다.**

그리고 `icp_node`는 `/livox/lidar`를 **거리 제한 없이** 받습니다 (MID-360 최대 70 m).
실내에서 열린 문·창문으로 나간 점은 맵에 대응점이 없습니다.

```
스캔의 10%가 맵에서 평균 12m 떨어져 있으면
  → 0.10 × 12² = 14.4
  → 나머지 90%가 완벽히 정렬돼도 fitness ≈ 14.5
```

**결과: `fitness_score_thre`가 수학적으로 도달 불가능해집니다.**
SC가 정답을 줘도 성공 판정이 안 납니다.

**해법 (둘 중 하나)**
- **정석 (C++ 한 줄)**: `icp.getFitnessScore(max_correspondence_distance)` — 짝이 잡힌 점만 평균.
  단 "1%만 맞아도 점수가 좋게" 나올 수 있으니 **짝 비율 조건**을 함께 둘 것
- **우회 (파이썬, 재빌드 없음)**: ICP 입력만 반경 클리핑한 토픽을 만들어 remap.
  **반경은 맵이 확실히 덮는 범위(20~30 m)로** — 좁게 자르면 정렬 품질이 나빠집니다

> **원본 `/livox/lidar`는 자르면 안 됩니다.** SLAM(FAST-LIO / SC-LIO-SAM)은
> 멀리 볼수록 좋고, `fastlio_relocalization.yaml`도 `det_range: 100.0`을 기대합니다.

### 8-4. `converged_count_thre = 40`은 연속 조건입니다

`fitness < 임계값`인 상태가 **연속 40프레임(10 Hz 기준 약 4초)** 유지돼야 성공 처리되고,
한 번이라도 실패하면 0으로 리셋됩니다. `sc_localizer`의 후보당 대기는 12초입니다.

또한 매 프레임 **같은 초기 추정에서 ICP를 새로 돌립니다** (앞 결과를 이어받지 않음).
즉 정밀도를 높이는 게 아니라 **"40번 독립 시도가 전부 성공하는가"** 라는 안정성 검사입니다.

### 8-5. 기타 운영 주의

- `localization.launch.py`의 `map_pcd`와 `fastlio_relocalization.yaml`의
  `prior_map_path`가 **같은 PCD**를 가리켜야 합니다.
- 런타임에서는 `livox_ros_driver2` 노드를 실행하지 않고 UDP 직접 파서를 사용합니다.
- `livox_udp_pointcloud2.py`의 host IP / point port는 현재 상수입니다.
  네트워크를 바꾸면 스크립트 값도 확인해야 합니다.
- Python UDP 파서는 부하가 큽니다. C++ 이식 또는 필터링 튜닝이
  장시간 실주행 전 우선 과제입니다 ([TODO](TODO.md)).
- **코드를 고쳤으면 `colcon build` 후 실행하세요.** `ros2 run`/`ros2 launch`는
  `install/` 사본을 실행하므로, `--symlink-install`을 쓰지 않으면 `src/` 수정이
  반영되지 않습니다:
  ```bash
  grep -c apply_visibility $(ros2 pkg prefix alm_navigation)/lib/alm_navigation/sc_build_db.py
  # 0 이면 옛 스크립트로 DB 를 만든 것 → 재빌드 후 DB 재생성
  ```

---

## 9. 자주 보는 문제

| 증상 | 확인 |
|---|---|
| `/livox/lidar` 없음 | Jetson IP `192.168.1.5`, LiDAR IP/포트, UDP 수신 여부 |
| `/livox/imu` 없음 | `MID360_config.json`의 host IMU port `56401`, 네트워크 |
| `/scan` 비어 있음 | LiDAR 높이, `pointcloud_to_scan` 높이 필터 |
| `/mcu/state` 없음 | `/dev/ttyTHS1`, baud `115200`, 권한, STM32 프로토콜 |
| TF 충돌 | 주행 모드에서는 EKF off, FAST-LIO가 `odom->base_link` 담당 |
| Nav2가 odom을 못 봄 | `nav2.yaml`의 `odom_topic`은 `/Odometry` |
| 로봇이 안 움직임 | `/cmd_vel`, `/mcu/command`, `/drive_mode/effective`, e-stop, MCU fault |
| **매핑 중 `SC loop found!`가 안 뜸** | 루프를 실제로 닫았는지, `Scancontext.h`의 `SC_DIST_THRES`/`PC_MAX_RADIUS` |
| **`sc_localizer`가 "SC DB 없음"** | `sc_db:=` 경로, `sc_build_db.py` 실행 여부 |
| **SC 후보 `d=` 값이 전부 0.4 이상** | DB와 실제 스캔의 링 프로파일 불일치 → §4 가시성 필터, `--max-radius` |
| **SC 후보 `d=` 값이 전부 비슷** | 공간에 특징이 부족 → `--z-max`를 올려 상부 구조 포함 (천장 아래로) |
| **방향은 맞는데 위치만 수 m 이탈** | DB의 벽 투과 → §4 가시성 필터 켜고 DB 재생성 |
| **`ICP fitness score`가 크고 안 변함** | §8-1 맵 불일치 → §8-3 맵 밖 점. `/transformed_cloud`를 RViz로 확인 |
| **후보 5개 전부 ICP 미수렴** | §8-2 `max_correspondence_distance`, §8-4 연속 성공 조건 |

---

## 10. 관련 문서

| 문서 | 내용 |
|---|---|
| [README](../README.md) | 방식 A/B/C 비교, 전체 실행 명령 |
| [`SC_ICP_실험체크리스트.md`](SC_ICP_실험체크리스트.md) | 초기위치 정합 실패 원인 분리 실험 절차 |
| [`JETSON_SETUP.md`](JETSON_SETUP.md) | Jetson 처음부터 설치 |
| [`TODO.md`](TODO.md) | 남은 작업 |
| [`CHANGES.md`](CHANGES.md) | 작업 내역 |
