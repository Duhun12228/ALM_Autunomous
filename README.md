# ALM_Autunomous
ALM 동아리 자율주행팀 ROS 2 프로젝트입니다.

워크스페이스 경로는 `ALM_auto_ws/`입니다.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ALM_auto_ws
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## Packages

- `alm_description`: 로봇 URDF, mesh, RViz 설정
- `alm_bringup`: 실제 로봇 실행 launch/config 진입점
- `alm_navigation`: SLAM, localization, Nav2 설정과 map
- `alm_sensors`: 3D LiDAR, IMU 드라이버 bringup, calibration, 센서 설정
- `alm_base_control`: `/cmd_vel` 안전 처리, 속도 제한, drive mode 관리
- `alm_mcu_interface`: Jetson과 MCU 사이 serial/CAN/UART 통신 인터페이스
- `alm_msgs`: Jetson-MCU 통신용 커스텀 메시지

## Architecture

이 프로젝트는 Jetson에서 ROS 2 상위 주행 스택을 실행하고, MCU가 실제 모터 제어와 저수준 실시간 제어를 담당하는 구조를 기준으로 합니다.

```text
3D LiDAR / IMU
  -> /points, /imu/data
alm_sensors
  -> filtering / conversion / calibration
SLAM / localization / Nav2 / teleop
  -> /cmd_vel
alm_base_control
  -> /mcu/command
alm_mcu_interface
  -> serial / CAN / UART
MCU
  -> encoder, motor, battery, fault feedback
alm_mcu_interface
  -> /mcu/state, /odom, /diagnostics
```

Jetson은 3D LiDAR와 IMU를 받아 SLAM, localization, Nav2를 실행하고, `geometry_msgs/Twist` 기반 상위 명령과 안전 상태를 MCU로 보냅니다. MCU는 4륜 독립조향 운동학, 모터 PID, 엔코더 처리, 통신 timeout 정지를 담당합니다.


## Workspace build
cd ~/ALM_Autunomous/ALM_auto_ws
colcon build --packages-select alm_sensors

## Source workspace
source install/setup.bash

## Launch LiDAR + EBIMU
ros2 launch alm_sensors sensors.launch.py

## 오류 발생 시
ebimu 포트나 baudrate가 다른 경우 :

ros2 launch alm_sensors sensors.launch.py ebimu_port:=/dev/ttyUSB0 ebimu_baudrate:=115200

권한 에러가 나는경우 :
sudo chmod 666 /dev/ttyUSB0
