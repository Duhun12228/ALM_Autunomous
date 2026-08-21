# ALM_Autunomous
ALM 동아리 자율주행팀 ROS 2 프로젝트입니다. 4륜 독립조향(2축 조향 + 4구동) 플랫폼카로
스마트팜/스마트팩토리에서 **3D LiDAR-Inertial SLAM → FAST-LIO 위치추정 → Nav2 경로계획 →
제어(STM32)**까지 자율주행하는 스택입니다.

워크스페이스 경로는 `ALM_auto_ws/`입니다. ROS 2 **Humble** 기준.

> **측위 아키텍처 전환**: 기존 2D(slam_toolbox+AMCL+EKF) → **3D LIO**(FAST-LIO2 매핑 +
> FAST-LIO-Localization 측위)로 전환했습니다. 주행 스택(base_control/mcu/STM32)은 그대로입니다.
> 세 가지 측위 방식을 브랜치로 병렬 개발 중: `main`(현행 ICP 재측위) ·
> `dev/fastlio2-sc`(FPFH+TEASER++ 자동초기화) · `dev/sc-lio-sam`(루프클로저 SLAM).
>
> **이 브랜치(`dev/fastlio2-sc`) = 방식 B**: 초기위치를 사람이 지정(RViz 2D Pose
> Estimate)하는 대신 **FPFH+TEASER++로 전역 정합**합니다. 맵의 FPFH DB를 1회 생성해
> 두면, 부팅 시 현재 스캔과 맵의 특징 대응점으로 초기 자세를 찾고 지역 GICP로
> 정밀화한 뒤 `/icp_result`를 발행합니다.

## 측위 3방식 (브랜치별 병렬 개발)

| 브랜치 | 방식 | 매핑(맵 생성) | 초기위치 자동특정 | 성격 |
|---|---|---|---|---|
| `main` | A | FAST-LIO2 | 수동 (RViz 2D Pose Estimate / `initial_*`) | 단순·안정, 레퍼런스 |
| **`dev/fastlio2-sc`** | **B** | **FAST-LIO2** | **FPFH+TEASER++ 자동** | **전역 강인 정합 + 지역 GICP** |
| `dev/sc-lio-sam` | C | SC-LIO-SAM(루프클로저) | Scan Context 자동 | 넓은 공간 드리프트 보정, GTSAM 빌드 필요 |

**← 이 브랜치 = 방식 B (`dev/fastlio2-sc`)**. 세 방식 공통: 추적(pose tracking)은
FAST-LIO, 경로계획은 Nav2, 구동은 STM32. 방식 A→B→C 순으로 무겁고 정교해집니다.

## 파이프라인

```
Livox MID-360 (3D LiDAR + 내장 6축 IMU)
  └ alm_sensors (UDP 직접 파싱, Livox 드라이버 노드 미사용)
      ├ livox_udp_pointcloud2 → /livox/lidar (PointCloud2 + per-point time 필드)
      │                         + 자기가림 마스크(LiDAR 뒤 적재물 점 제거)
      ├ livox_udp_imu         → /livox/imu (내장 6축)
      └ pointcloud_to_scan    → /scan (2D costmap 관측용)

[매핑]  FAST-LIO2 (fastlio_mapping): /livox/lidar + /livox/imu → 3D 누적점군 (odom 프레임)
        → /map_save → maps/alm_3d_map.pcd   → pcd2pgm → maps/alm_map.pgm/yaml (2D 맵)

[측위]  FAST-LIO-Localization:
        FPFH 대응점 → TEASER++ 전역 SE(3) → overlap 검증 → 지역 GICP → /icp_result
          → transform_publisher → TF map→odom
        fastlio_localization → TF odom→base_link, /Odometry (실시간 추적)
        ※ AMCL + robot_localization EKF 를 대체

Nav2 (odom_topic=/Odometry) → /cmd_vel
  ├ planner   SmacPlannerHybrid (Hybrid-A*, Reeds-Shepp, R_min=1.643 m)
  ├ smoother  ConstrainedSmoother (Ceres 곡률/장애물 제약)
  └ controller MPPI (샘플링 MPC) ※ 연결은 커스텀 BT(navigate_*_w_smoothing.xml)
alm_base_control · command_manager: /cmd_vel + /drive_mode
      → 모드해석(auto→normal/spin/crab) + ALIGN(경로 헤딩 정렬) + 안전게이팅
      → /mcu/command (McuCommand)
alm_mcu_interface · mcu_bridge: /mcu/command ⇄ STM32 (UART) → /wheel_odom, /mcu/state, /joint_states
STM32: 역기구학(twist→2조향+4구동) · 모터 PID · 엔코더 정기구학(→odom)
```

TF: `map →(TEASER/GICP+transform_publisher) odom →(FAST-LIO) base_link →(URDF) 4×steer/wheel`,
`base_link →(static) livox_frame`. `use_sim_time=false` (실차).
매핑 모드에선 EKF(`/wheel_odom`+IMU)가 odom→base_link 를 담당하고, 주행 모드에선
FAST-LIO 가 담당하므로 EKF 를 끕니다(`use_ekf:=false`, TF 충돌 방지). 단 **맵(.pcd) 자체는
LiDAR+IMU 만으로 만들며 엔코더는 관여하지 않습니다.**

## FPFH+TEASER++ 초기측위 상세

이 브랜치의 초기측위는 초기 위치 후보를 외부에서 받지 않고, 정지 상태의 현재 LiDAR
스캔과 prior map 전체를 직접 비교합니다. `teaser_fpfh_localizer`의 처리 순서는 다음과
같습니다.

1. `/livox/lidar` 10프레임을 누적합니다. 누적 중 로봇이 움직이면 서로 다른 자세의 점이
   섞이므로 반드시 정지해야 합니다.
2. 수평거리 `0.5~10.0 m`, 높이 `-0.35~1.0 m`만 남긴 후 `0.5 m` voxel로 다운샘플합니다.
3. 반경 `1.0 m`에서 normal, 반경 `2.5 m`에서 FPFH를 계산합니다.
4. 현재 스캔과 DB 사이에서 mutual nearest-neighbor와 ratio test(`0.95`)를 모두 만족하는
   대응점을 최대 400개 선택합니다. 20개 미만이면 새 스캔으로 다시 시작합니다.
5. TEASER++가 outlier를 제거하고 전역 SE(3) 자세를 구합니다. maximum clique가 6개
   미만이면 거절합니다. **6은 최종 성공 기준이 아니라 후단 검증에 들어가기 위한 최소값**입니다.
6. 지상 로봇 제약으로 `|roll|≤15°`, `|pitch|≤15°`, `-1.0≤z≤1.0 m`를 검사합니다.
7. TEASER 결과를 원본 prior map과 독립적으로 대조합니다. `0.5 m` 이내 대응점이 20% 이상이고
   해당 점 RMSE가 `0.35 m` 미만이어야 합니다.
8. 후보 주변 반경 `12 m`의 local map에서 GICP를 수행합니다. 최대 대응거리 `1.0 m`, 최대
   60회 반복, fitness `<0.30`과 동일한 overlap/RMSE 조건을 모두 만족해야 합니다.
9. 별도로 다시 누적한 스캔에서 같은 결과가 2회 연속 나와야 합니다. 두 결과 차이는 이동
   `≤0.50 m`, 회전 `≤5°`여야 하며, 통과 시에만 `/icp_result`를 한 번 발행합니다.

주요 토픽은 다음과 같습니다.

| 토픽 | 형식 | 용도 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 누적할 실시간 LiDAR 입력 |
| `/teaser_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 자세 제한과 clique를 통과한 TEASER 중간 후보 |
| `/teaser_aligned_cloud` | `sensor_msgs/PointCloud2` | GICP까지 통과한 정합 점군 디버그 출력 |
| `/icp_result` | `geometry_msgs/PoseWithCovarianceStamped` | 2회 일관성 검증까지 통과한 최종 초기 자세 |

### 맵 DB 일관성

DB는 `fpfh_map.meta`, `fpfh_map_points.pcd`, `fpfh_map_normals.pcd`,
`fpfh_map_fpfh.pcd` 네 파일로 구성됩니다. `.meta`에는 원본 PCD fingerprint와 feature
전처리 값이 기록됩니다. 다음 항목 중 하나라도 달라지면 기존 DB를 재사용하지 말고 다시
생성해야 합니다.

- `alm_3d_map.pcd` 내용
- `voxel`, normal/FPFH 반경
- `z-min`, `z-max`, curvature 필터
- LiDAR 자기가림 또는 다른 센서 전처리

launch의 `map_pcd`, FAST-LIO 설정의 `prior_map_path`, `fpfh_db_prefix`는 반드시 같은 맵
세트를 가리켜야 합니다. 기본적으로 fingerprint 검증이 켜져 있어 서로 다른 맵이면 노드가
시작 단계에서 실패합니다.

### 현재 검증 상태와 한계

2026-08-05 실차 로그 기준 DB는 1,414개 feature이고, 한 번의 누적 스캔에서는 약 398개
feature와 평균 26.8개의 FPFH 대응점이 생성됐습니다. 79회 시도 중 clique 6 이상은 9회였지만
8회는 비현실적인 roll/pitch/z로 거절됐고, 나머지 1회도 map overlap이 3.1%라 거절됐습니다.
GICP까지 진입하거나 `/icp_result`를 발행한 시도는 없었습니다.

따라서 현재 구현은 오답 차단 검증은 정상 동작하지만, 긴 평행 벽이 많은 환경에서 올바른
전역 후보를 아직 안정적으로 만들지 못하는 **실험 단계**입니다. clique를 5 이하로 더 낮추면
오답 후보만 대량으로 후단에 유입될 가능성이 높습니다. 다음 개선 우선순위는 임계값 완화가
아니라 지상 로봇에 맞는 `x/y/yaw` 평면 제약과 FPFH 대응점 품질 개선입니다.

## Packages

- `alm_description`: 4WIS URDF(xacro, CAD 실측), robot_state_publisher, RViz
- `alm_sensors`: Livox MID-360 **UDP 직접 파싱**(livox_udp_pointcloud2/imu, per-point time 포함)
  + PointCloud→Scan. 런타임에서 livox_ros_driver2 드라이버 노드는 쓰지 않음.
  **자기가림 마스크**: LiDAR 뒤 적재물을 `/livox/lidar` 발행 전에 제거(방위 center±width
  + 수평거리)하므로 FAST-LIO·FPFH/GICP·Nav2 costmap·`/scan` 에 동시에 적용된다.
- `alm_navigation`: **FAST-LIO2 매핑**(slam.launch) · **FAST-LIO-Localization 측위**(localization.launch)
  · **pcd2pgm**(3D→2D) · Nav2 설정/launch · EKF(매핑용) · map · rviz. `map_publisher.py`(맵 확인용).
  [방식 B] **FPFH+TEASER++**: `fpfh_map_builder`(맵 feature DB) ·
  `teaser_fpfh_localizer`(전역 정합/검증/지역 GICP).
  **경로계획**: Hybrid-A* → ConstrainedSmoother → MPPI. 파라미터는 STM32 CONS 와
  URDF 에서 유도하며 `nav2_kinematic_check.py` 가 정합성을 검사한다 → `docs/nav2_planning.md`
- `alm_base_control`: `command_manager` — 모드 선택 + 속도/가속 제한 + e-stop
  + **조향각 한계**(normal 모드 `|wz| ≤ |vx|/R_min`, R_min 은 `fourwis_encode` 가 런타임 계산)
- `alm_mcu_interface`: `mcu_bridge` — Jetson↔STM32 UART, `docs/uart_protocol.md`
- `alm_msgs`: `McuCommand`(다운링크), `McuState`(업링크, 2조향+4구동 피드백)
- `alm_bringup`: robot/slam/navigation 최상위 launch
- `thirdparty/Fast-LIO2-Localization`(vendored, PolarisXQ): `fast_lio`(fastlio_mapping) +
  `icp_relocalization`(FPFH DB/TEASER++/GICP/transform_publisher)
- `thirdparty/TEASER-plusplus`(MIT-SPARK v2.0 고정): 강인 전역 자세 추정

## 사전 설치 (별도 의존성)

```bash
sudo apt install ros-humble-robot-localization \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pcl-ros pcl-tools \
  ros-humble-joint-state-publisher-gui python3-serial
pip3 install numpy pyyaml pillow    # pcd2pgm / map_publisher / 맵 렌더링
```
- **FAST-LIO(측위 엔진)**: `ALM_auto_ws/src/thirdparty/Fast-LIO2-Localization` 에 vendoring 되어
  있어 별도 clone 불필요 — 워크스페이스와 함께 빌드됩니다.
- **TEASER++**도 소스가 vendoring 되어 있습니다. 깨끗한 환경의 첫 빌드에서는 고정된
  PMC/Spectra 하위 의존성을 내려받기 위한 네트워크 연결이 한 번 필요합니다.
- **Livox 런타임 드라이버 불필요**: 센서 데이터는 UDP 직접 파싱으로 받으므로
  livox_ros_driver2 노드는 실행하지 않습니다. 단 vendored FAST-LIO/ICP 코드가
  `livox_ros_driver2` 메시지 헤더를 빌드 의존성으로 갖고 있어, 깨끗한 Jetson에서
  전체 워크스페이스를 빌드할 때는 Livox-SDK2/livox_ros_driver2 빌드 의존성이 필요할 수 있습니다.
  (네트워크: 호스트 IP `192.168.1.5`, LiDAR `192.168.1.147`, 포트 56301/56401).

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ALM_Autunomous/ALM_auto_ws
colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOC=OFF -DBUILD_TEASER_FPFH=OFF \
  -DBUILD_TEASER_IO=OFF
source install/setup.bash
```

## 실행

경로 프리픽스: `WS=~/ALM_Autunomous/ALM_auto_ws`, `MAPS=$WS/src/alm_navigation/maps`.

```bash
# 1) 상시 하드웨어 스택 (센서+EKF+제어+MCU)
ros2 launch alm_bringup robot.launch.py

# 1.5) 자기가림 마스크 튜닝 (적재물 형상이 바뀌었을 때만)
ros2 launch alm_sensors lidar.launch.py mask_debug:=/livox/lidar_masked
#    RViz 에 /livox/lidar(흰색) + /livox/lidar_masked(빨강) 동시 표시 →
#    적재물만 빨갛게 덮이도록 mask_center_deg/mask_width_deg/mask_max_range 조정
#    ##중요## 마스크를 바꾸면 아래 2)~2.5) 를 다시 돌려 맵과 FPFH DB 를 재생성할 것.
#    스캔에서만 점이 빠지고 prior map 에는 남아 있으면 정합이 오히려 나빠진다.

# 2) 매핑 (FAST-LIO2 3D SLAM)
ros2 launch alm_sensors lidar.launch.py          # 터미널1: 센서(/livox/lidar,/livox/imu)
ros2 launch alm_navigation slam.launch.py rviz:=true   # 터미널2: FAST-LIO2 (RViz Fixed Frame=odom)
#    LiDAR/로봇으로 천천히 한 바퀴(루프 닫기) 후 3D 맵 저장:
ros2 service call /map_save std_srvs/srv/Trigger       # → $MAPS/alm_3d_map.pcd
#    3D pcd → 2D occupancy 맵 (벽만 잡히게 z밴드 튜닝):
ros2 run alm_navigation pcd2pgm.py --pcd $MAPS/alm_3d_map.pcd --out $MAPS/alm_map \
  --resolution 0.05 --z-min 0.3 --z-max 0.8            # → $MAPS/alm_map.pgm/yaml

# 2.5) [방식 B] FPFH 맵 DB 생성 (맵 갱신 때마다 1회, 오프라인)
ros2 run icp_relocalization fpfh_map_builder \
  --map $MAPS/alm_3d_map.pcd --output-prefix $MAPS/fpfh_map \
  --voxel 0.5 --normal-radius 1.0 --feature-radius 2.5 \
  --z-min -0.35 --z-max 1.0 --max-features 20000 --threads 4
#    생성물: fpfh_map.meta, fpfh_map_{points,normals,fpfh}.pcd
#    맵 또는 위 전처리 값을 바꾸면 DB를 반드시 다시 생성한다.

# 3) 측위만 검증 (FAST-LIO-Localization + FPFH/TEASER++ 자동초기화)
ros2 launch alm_sensors lidar.launch.py                # 센서
ros2 launch alm_navigation localization.launch.py      # TEASER++ + GICP + transform_publisher + fastlio
#    10프레임 누적 동안 로봇 정지 → FPFH 대응점 → TEASER++ → overlap 검증 → 지역 GICP
#    → 새 누적 스캔 2회에서 자세가 일치하면 /icp_result 발행.
ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=$MAPS/alm_map.yaml   # /map 발행
rviz2 -d /home/kdh/ALM_Autunomous/ALM_auto_ws/install/alm_navigation/share/alm_navigation/rviz/localization.rviz

# 측위 로그를 터미널에 표시하면서 파일에도 전부 저장
mkdir -p $WS/logs
ros2 launch alm_navigation localization.launch.py 2>&1 | \
  tee "$WS/logs/localization_$(date +%Y%m%d_%H%M%S).log"

# 4) 자율주행 (측위 + Nav2). 주행 모드에선 EKF off
ros2 launch alm_bringup navigation.launch.py map:=$MAPS/alm_map.yaml \
  map_pcd:=$MAPS/alm_3d_map.pcd fpfh_db_prefix:=$MAPS/fpfh_map
#    FPFH+TEASER++ 초기위치 완료 후 RViz에서 Nav2 Goal 지정
#    또는 auto 모드로: ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1

# 5) [수동] 키보드 텔레옵 — ROS 경유. 자율과 같은 경로로 STM32 까지 내려간다.
#    시리얼 포트는 mcu_bridge 하나만 소유한다(포트 이중 점유 금지).
ros2 run alm_base_control keyboard_teleop.py
#    t=동작권 잡기(teleop), WASD/qe 조작, r=반납(auto), space=비상정지, c=해제
#    기본 소유자는 자율(auto). 텔레옵이 명령을 끊으면 정지 유지(HELD).
#    ⚠ 바퀴가 실제로 돕니다. 잭업 상태에서 먼저 확인할 것. 상세 → docs/control_arbitration.md
```

> 주의: `localization.launch.py` 의 `map_pcd` 인자와
> `config/fastlio_relocalization.yaml` 의 `prior_map_path` 는 같은 3D PCD 를 가리켜야 합니다.
> `fpfh_db_prefix`도 이 PCD에서 생성한 DB여야 합니다. fingerprint 또는 전처리 값이
> 다르면 로컬라이저가 시작 단계에서 명시적으로 거절합니다.

로그의 주요 태그는 `[ACCUM]`(프레임 누적), `[FPFH]`(특징 계산), `[MATCH]`(대응점),
`[TEASER]`(전역 후보와 clique/overlap), `[GICP]`(지역 정밀화), `[CONSISTENCY]`(연속 결과
일치) 순서입니다. Fast-LIO의 `Waiting for initial pose...`는 오류가 아니라 `/icp_result`를
기다리는 정상 상태입니다.

### 주행 모드 (`/drive_mode`)
`normal`(전후+회전) · `spin`(제자리 회전) · `crab`(게걸음, 기본 비활성) · `auto`(자동 선택).
auto 는 Nav2 의 `/cmd_vel`(vx+wz)을 보고 normal↔spin 을 자동 전환합니다(참고 레포 로직 포팅).

여기에 더해 `command_manager` 는 **`/plan` 을 직접 보고** 경로가 요구하는 헤딩과 실제
헤딩이 60° 넘게(0.6 s 지속) 벌어지면 스스로 `spin` 을 걸어 헤딩을 고칩니다(`ALIGN`).
전역 플래너는 `R_min` 원호/직선만 이어 붙여 **제자리 회전을 표현하지 못하므로**, 4륜
독립조향의 제자리 회전을 실제로 쓰는 경로는 이것뿐입니다.
설계 근거는 `alm_base_control/scripts/path_align.py` docstring,
검증 결과는 `docs/control_pipeline.md` §6.8 · §7.3.

`normal` 모드는 자동차형이라 **최소 선회반경 1.643 m**(내측 전륜 30°, 후륜 50% 역조향
포함) 아래로는 못 돕니다. 그보다 급한 요청은 `command_manager` 가
`|wz| ≤ |vx|/R_min` 으로 접어 조향각이 포화되지 않게 합니다 — 그래야
`/mcu/command` 의 `cmd_vel` 과 `steer_deg` 가 같은 이야기를 합니다.
상세 → `docs/nav2_planning.md` §3

## 문서
- **Jetson 처음부터 설치·실행 → `docs/JETSON_SETUP.md`**
- 매핑→저장→자율주행 운영 → `docs/OPERATION_GUIDE.md`
- **Nav2 경로계획(Hybrid-A*)·조향각 한계 → `docs/nav2_planning.md`**
- **제어 루프 현황 분석(피드백 끊긴 지점·해결 방향) → `docs/control_feedback_analysis.md`**
  (처음 보는 사람용 전체 파이프라인 해설 포함)
- **Jetson→STM32 UART 연동·확인 절차 → `docs/uart.md`** (변경사항 정리 + 제어단 점검 체크리스트)
- UART 프로토콜 규격 v2 → `ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md`
- 실차 전 확인/수정할 값 → `SETUP_CHECKLIST.md`
- 작업 내역 → `docs/CHANGES.md`
- **남은 작업(TODO) → `docs/TODO.md`**

## Architecture 상세

Jetson 은 3D LiDAR/IMU 로 SLAM·측위·Nav2 를 실행하고 `geometry_msgs/Twist` 상위 명령을
STM32 로 보냅니다. STM32 는 2축 조향 + 4구동 역/정기구학, 모터 PID, 엔코더 처리, 통신
timeout 정지를 담당합니다. UART 프레임 규격은
`ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md` 참고.
