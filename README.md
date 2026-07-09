# ALM_Autunomous
ALM 동아리 자율주행팀 ROS 2 프로젝트입니다. 4륜 독립조향(2축 조향 + 4구동) 플랫폼카로
스마트팜/스마트팩토리에서 SLAM → AMCL 위치추정 → Nav2 경로계획 → 제어(STM32)까지
자율주행하는 스택입니다.

워크스페이스 경로는 `ALM_auto_ws/`입니다. ROS 2 **Humble** 기준.

## 파이프라인

```
Livox MID-360 (3D LiDAR + 내장 6축 IMU)
  └ livox_ros_driver2 → /livox/lidar(PointCloud2), /livox/imu(Imu)
      ├ pointcloud_to_scan → /scan
      └ imu_relay          → /imu/data (orientation 무효화)
robot_localization EKF: /wheel_odom + /imu/data → /odometry/filtered, TF odom→base_link
slam_toolbox(매핑) / AMCL(측위): /scan + TF → /map, TF map→odom
Nav2 (planner A*/NavFn, controller DWB) → /cmd_vel
alm_base_control · command_manager: /cmd_vel + /drive_mode
      → 모드해석(auto→normal/spin/crab) + 안전게이팅 → /mcu/command (McuCommand)
alm_mcu_interface · mcu_bridge: /mcu/command ⇄ STM32 (UART) → /wheel_odom, /mcu/state, /joint_states
STM32: 역기구학(twist→2조향+4구동) · 모터 PID · 엔코더 정기구학(→odom)
```

TF: `map →(slam/amcl) odom →(EKF) base_link →(URDF) 4×steer/wheel`, `base_link →(static) livox_frame`.
`use_sim_time=false` (실차).

## Packages

- `alm_description`: 4WIS URDF(xacro, CAD 실측), robot_state_publisher, RViz
- `alm_sensors`: Livox MID-360 드라이버 bringup + PointCloud→Scan + IMU relay
- `alm_navigation`: EKF, slam_toolbox, AMCL/Nav2 설정과 launch, map
- `alm_base_control`: `command_manager` — 모드 선택 + 속도/가속 제한 + e-stop
- `alm_mcu_interface`: `mcu_bridge` — Jetson↔STM32 UART, `docs/uart_protocol.md`
- `alm_msgs`: `McuCommand`(다운링크), `McuState`(업링크, 2조향+4구동 피드백)
- `alm_bringup`: robot/slam/navigation 최상위 launch

## 사전 설치 (별도 의존성)

```bash
sudo apt install ros-humble-robot-localization ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-joint-state-publisher-gui python3-serial
```
- **livox_ros_driver2**: 소스 빌드 필요 — https://github.com/Livox-SDK/livox_ros_driver2
  (Livox-SDK2 먼저 설치). 빌드 후 이 워크스페이스와 함께 소싱하세요.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ALM_Autunomous/ALM_auto_ws
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 실행

```bash
# 1) 상시 하드웨어 스택 (센서+EKF+제어+MCU). 단독 실행도 가능
ros2 launch alm_bringup robot.launch.py

# 2) 매핑 (상시 스택 + slam_toolbox)
ros2 launch alm_bringup slam.launch.py
#    맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/my_map

# 3) 자율주행 (상시 스택 + AMCL + Nav2)
ros2 launch alm_bringup navigation.launch.py \
  map:=~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/my_map.yaml
#    RViz 에서 2D Pose Estimate 로 초기위치 → Nav2 Goal 지정
#    또는 auto 모드로: ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

### 주행 모드 (`/drive_mode`)
`normal`(전후+회전) · `spin`(제자리 회전) · `crab`(게걸음, 기본 비활성) · `auto`(자동 선택).
auto 는 Nav2 의 `/cmd_vel`(vx+wz)을 보고 normal↔spin 을 자동 전환합니다(참고 레포 로직 포팅).

## 문서
- **Jetson 처음부터 설치·실행 → `docs/JETSON_SETUP.md`**
- 매핑→저장→자율주행 운영 → `docs/OPERATION_GUIDE.md`
- 실차 전 확인/수정할 값 → `SETUP_CHECKLIST.md`
- 작업 내역 → `docs/CHANGES.md`

## Architecture 상세

Jetson 은 3D LiDAR/IMU 로 SLAM·측위·Nav2 를 실행하고 `geometry_msgs/Twist` 상위 명령을
STM32 로 보냅니다. STM32 는 2축 조향 + 4구동 역/정기구학, 모터 PID, 엔코더 처리, 통신
timeout 정지를 담당합니다. UART 프레임 규격은
`ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md` 참고.
