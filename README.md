# ALM_Autunomous
ALM 동아리 자율주행팀 ROS 2 프로젝트입니다. 4륜 독립조향(2축 조향 + 4구동) 플랫폼카로
스마트팜/스마트팩토리에서 **3D LiDAR-Inertial SLAM → FAST-LIO 위치추정 → Nav2 경로계획 →
제어(STM32)**까지 자율주행하는 스택입니다.

워크스페이스 경로는 `ALM_auto_ws/`입니다. ROS 2 **Humble** 기준.

> **측위 아키텍처 전환**: 기존 2D(slam_toolbox+AMCL+EKF) → **3D LIO**(FAST-LIO2 매핑 +
> FAST-LIO-Localization 측위)로 전환했습니다. 주행 스택(base_control/mcu/STM32)은 그대로입니다.
> 세 가지 측위 방식을 브랜치로 병렬 개발 중: `main`(현행 ICP 재측위) ·
> `dev/fastlio2-sc`(Scan Context 자동초기화) · `dev/sc-lio-sam`(루프클로저 SLAM).

## 파이프라인

```
Livox MID-360 (3D LiDAR + 내장 6축 IMU)
  └ alm_sensors (UDP 직접 파싱, Livox 드라이버 노드 미사용)
      ├ livox_udp_pointcloud2 → /livox/lidar (PointCloud2 + per-point time 필드)
      ├ livox_udp_imu         → /livox/imu (내장 6축)
      └ pointcloud_to_scan    → /scan (2D costmap 관측용)

[매핑]  FAST-LIO2 (fastlio_mapping): /livox/lidar + /livox/imu → 3D 누적점군 (odom 프레임)
        → /map_save → maps/alm_3d_map.pcd   → pcd2pgm → maps/alm_map.pgm/yaml (2D 맵)

[측위]  FAST-LIO-Localization:
        icp_node (현재스캔 ↔ prior map.pcd ICP, 초기위치 1회) → /icp_result
          → transform_publisher → TF map→odom
        fastlio_localization → TF odom→base_link, /Odometry (실시간 추적)
        ※ AMCL + robot_localization EKF 를 대체

Nav2 (planner/controller/bt, odom_topic=/Odometry) → /cmd_vel
alm_base_control · command_manager: /cmd_vel + /drive_mode
      → 모드해석(auto→normal/spin/crab) + 안전게이팅 → /mcu/command (McuCommand)
alm_mcu_interface · mcu_bridge: /mcu/command ⇄ STM32 (UART) → /wheel_odom, /mcu/state, /joint_states
STM32: 역기구학(twist→2조향+4구동) · 모터 PID · 엔코더 정기구학(→odom)
```

TF: `map →(icp/transform_publisher) odom →(FAST-LIO) base_link →(URDF) 4×steer/wheel`,
`base_link →(static) livox_frame`. `use_sim_time=false` (실차).
매핑 모드에선 EKF(`/wheel_odom`+IMU)가 odom→base_link 를 담당하고, 주행 모드에선
FAST-LIO 가 담당하므로 EKF 를 끕니다(`use_ekf:=false`, TF 충돌 방지). 단 **맵(.pcd) 자체는
LiDAR+IMU 만으로 만들며 엔코더는 관여하지 않습니다.**

## Packages

- `alm_description`: 4WIS URDF(xacro, CAD 실측), robot_state_publisher, RViz
- `alm_sensors`: Livox MID-360 **UDP 직접 파싱**(livox_udp_pointcloud2/imu, per-point time 포함)
  + PointCloud→Scan. 런타임에서 livox_ros_driver2 드라이버 노드는 쓰지 않음.
- `alm_navigation`: **FAST-LIO2 매핑**(slam.launch) · **FAST-LIO-Localization 측위**(localization.launch)
  · **pcd2pgm**(3D→2D) · Nav2 설정/launch · EKF(매핑용) · map · rviz. `map_publisher.py`(맵 확인용).
- `alm_base_control`: `command_manager` — 모드 선택 + 속도/가속 제한 + e-stop
- `alm_mcu_interface`: `mcu_bridge` — Jetson↔STM32 UART, `docs/uart_protocol.md`
- `alm_msgs`: `McuCommand`(다운링크), `McuState`(업링크, 2조향+4구동 피드백)
- `alm_bringup`: robot/slam/navigation 최상위 launch
- `thirdparty/Fast-LIO2-Localization`(vendored, PolarisXQ): `fast_lio`(fastlio_mapping) +
  `icp_relocalization`(icp_node/transform_publisher/sac_ia_gicp)

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
- **Livox 런타임 드라이버 불필요**: 센서 데이터는 UDP 직접 파싱으로 받으므로
  livox_ros_driver2 노드는 실행하지 않습니다. 단 vendored FAST-LIO/ICP 코드가
  `livox_ros_driver2` 메시지 헤더를 빌드 의존성으로 갖고 있어, 깨끗한 Jetson에서
  전체 워크스페이스를 빌드할 때는 Livox-SDK2/livox_ros_driver2 빌드 의존성이 필요할 수 있습니다.
  (네트워크: 호스트 IP `192.168.1.5`, LiDAR `192.168.1.147`, 포트 56301/56401).

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ALM_Autunomous/ALM_auto_ws
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 실행

경로 프리픽스: `WS=~/ALM_Autunomous/ALM_auto_ws`, `MAPS=$WS/src/alm_navigation/maps`.

```bash
# 1) 상시 하드웨어 스택 (센서+EKF+제어+MCU)
ros2 launch alm_bringup robot.launch.py

# 2) 매핑 (FAST-LIO2 3D SLAM)
ros2 launch alm_sensors lidar.launch.py          # 터미널1: 센서(/livox/lidar,/livox/imu)
ros2 launch alm_navigation slam.launch.py rviz:=true   # 터미널2: FAST-LIO2 (RViz Fixed Frame=odom)
#    LiDAR/로봇으로 천천히 한 바퀴(루프 닫기) 후 3D 맵 저장:
ros2 service call /map_save std_srvs/srv/Trigger       # → $MAPS/alm_3d_map.pcd
#    3D pcd → 2D occupancy 맵 (벽만 잡히게 z밴드 튜닝):
ros2 run alm_navigation pcd2pgm.py --pcd $MAPS/alm_3d_map.pcd --out $MAPS/alm_map \
  --resolution 0.05 --z-min 0.3 --z-max 0.8            # → $MAPS/alm_map.pgm/yaml

# 3) 측위만 검증 (FAST-LIO-Localization)
ros2 launch alm_sensors lidar.launch.py                # 센서
ros2 launch alm_navigation localization.launch.py      # icp + transform_publisher + fastlio
ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=$MAPS/alm_map.yaml   # /map 발행
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz   # 2D 트래킹 뷰
#    시작점이 맵 원점과 다르면 RViz "2D Pose Estimate"로 실제 위치 지정 → ICP 수렴(로그 converged)

# 4) 자율주행 (측위 + Nav2). 주행 모드에선 EKF off
ros2 launch alm_bringup navigation.launch.py map:=$MAPS/alm_map.yaml map_pcd:=$MAPS/alm_3d_map.pcd
#    RViz 에서 2D Pose Estimate 로 초기위치 → Nav2 Goal 지정
#    또는 auto 모드로: ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

> 주의: `localization.launch.py` 의 `map_pcd` 인자와
> `config/fastlio_relocalization.yaml` 의 `prior_map_path` 는 같은 3D PCD 를 가리켜야 합니다.
> 둘이 다르면 ICP 와 FAST-LIO 가 서로 다른 prior map 을 기준으로 동작할 수 있습니다.

### 주행 모드 (`/drive_mode`)
`normal`(전후+회전) · `spin`(제자리 회전) · `crab`(게걸음, 기본 비활성) · `auto`(자동 선택).
auto 는 Nav2 의 `/cmd_vel`(vx+wz)을 보고 normal↔spin 을 자동 전환합니다(참고 레포 로직 포팅).

## 문서
- **Jetson 처음부터 설치·실행 → `docs/JETSON_SETUP.md`**
- 매핑→저장→자율주행 운영 → `docs/OPERATION_GUIDE.md`
- 실차 전 확인/수정할 값 → `SETUP_CHECKLIST.md`
- 작업 내역 → `docs/CHANGES.md`
- **남은 작업(TODO) → `docs/TODO.md`**

## Architecture 상세

Jetson 은 3D LiDAR/IMU 로 SLAM·측위·Nav2 를 실행하고 `geometry_msgs/Twist` 상위 명령을
STM32 로 보냅니다. STM32 는 2축 조향 + 4구동 역/정기구학, 모터 PID, 엔코더 처리, 통신
timeout 정지를 담당합니다. UART 프레임 규격은
`ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md` 참고.
