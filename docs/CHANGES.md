# 작업 내역 정리

이 문서는 현재 워크스페이스 상태를 기준으로 정리한 변경 내역입니다.
초기에는 ROS 2 패키지 뼈대와 일부 센서 스크립트가 있었고, 이후 4WIS 실차 주행을
목표로 센서, 측위, Nav2, MCU 통신 계층이 추가되었습니다.

## 현재 아키텍처 요약

```text
Livox MID-360 UDP 직접 파싱
  -> FAST-LIO2 3D 매핑
  -> FAST-LIO-Localization 측위
  -> Nav2 경로계획/제어
  -> command_manager 안전 게이팅
  -> mcu_bridge UART
  -> STM32 2축 조향 + 4구동 제어
```

기존 2D `slam_toolbox + AMCL + EKF` 중심 구조는 현행 주행 경로가 아닙니다.
`slam_toolbox.yaml`, AMCL 파라미터 등 일부 잔여 설정은 남아 있지만,
운영 기준은 README와 이 문서의 FAST-LIO 경로입니다.

## alm_msgs

- `McuCommand.msg`
  - Jetson -> STM32 상위 명령.
  - `Twist`, `drive_mode`, `enable_motors`, `emergency_stop`, `sequence`.
- `McuState.msg`
  - STM32 -> Jetson 피드백.
  - 2축 조향각 `[front, rear]`, 4구동 휠 속도, odom pose, 배터리, fault 상태.

## alm_description

- 4WIS 로봇 URDF 추가.
- CAD 실측 기반 주요 값:
  - wheel radius `0.103`
  - wheel width `0.0488`
  - front_x `+0.6106`
  - rear_x `-0.3010`
  - half_track `0.500`
  - body `1.200 x 0.9872 x 0.450`
- `description.launch.py`, RViz 설정 추가.
- LiDAR 장착 위치는 URDF에 고정하지 않고 `lidar.launch.py`의 static TF 인자로 관리.

## alm_sensors

- Livox MID-360 UDP 직접 파싱 방식으로 전환.
- `livox_udp_pointcloud2.py`
  - UDP point packet 수신.
  - `/livox/lidar` PointCloud2 발행.
  - FAST-LIO용 per-point `time` 필드 포함.
- `livox_udp_imu.py`
  - MID-360 내장 6축 IMU UDP 수신.
  - `/livox/imu` 발행.
- `imu_relay.py`
  - `/livox/imu` -> `/imu/data`.
  - orientation covariance를 `-1`로 설정해 EKF가 가짜 orientation을 융합하지 않게 함.
- `pointcloud_to_scan.py`
  - `/livox/lidar` -> `/scan`.
  - Nav2 costmap/시각화용 2D scan 생성.
- `lidar.launch.py`
  - livox_ros_driver2 런타임 드라이버 노드 없이 위 노드들을 통합 실행.
  - 단 FAST-LIO/ICP 빌드에는 vendored livox_ros_driver2 메시지 헤더 의존성이 남아 있음.
  - `base_link -> livox_frame` static TF 발행.

## alm_navigation

- FAST-LIO2 매핑 launch 추가.
  - `slam.launch.py`
  - `/map_save` 서비스로 `maps/alm_3d_map.pcd` 저장.
- FAST-LIO-Localization launch 추가.
  - `localization.launch.py`
  - `icp_node`: 현재 scan과 prior PCD ICP 정합.
  - `transform_publisher`: `/icp_result` 기반 `map->odom` TF.
  - `fastlio_localization`: `odom->base_link`, `/Odometry`.
- Nav2 launch 재구성.
  - `navigation.launch.py`
  - `map_server`는 pcd2pgm 결과인 2D YAML/PGM 사용.
  - Nav2 odom topic은 `/Odometry`.
- `pcd2pgm.py`
  - FAST-LIO PCD를 Nav2용 2D occupancy map으로 변환.
- `map_publisher.py`
  - Nav2 없이 2D 맵을 `/map`으로 띄워 RViz 검증.
- `localization.rviz`, `fastlio_mapping.rviz` 추가.

## alm_base_control

- `command_manager.py` 추가.
  - `/cmd_vel`, `/drive_mode`, `/emergency_stop`, `/mcu/state` 구독.
  - auto 모드에서 normal/spin/crab 선택.
  - 속도/가속 제한.
  - cmd timeout, e-stop, MCU fault, odom watchdog 반영.
  - `/mcu/command` 발행.
- `base_control.yaml`에 속도 제한과 안전 파라미터 정리.

## alm_mcu_interface

- `mcu_bridge.py` 추가.
  - UART frame 송수신.
  - `/mcu/command` -> STM32.
  - STM32 state -> `/mcu/state`, `/wheel_odom`, `/joint_states`.
  - 기구학은 STM32 담당, Jetson은 전송 계층과 ROS topic 변환 담당.
- `docs/uart_protocol.md` 추가.
  - frame sync, CRC16-CCITT, Command 18 bytes, State 63 bytes.
  - STM32 역기구학/정기구학 구현 가이드 포함.

## alm_bringup

- `robot.launch.py`
  - description, sensors, EKF, base_control, MCU bridge 통합.
  - `use_ekf` 인자로 EKF on/off 가능.
- `slam.launch.py`
  - robot stack 위에 FAST-LIO2 매핑 launch 포함.
- `navigation.launch.py`
  - robot stack을 `use_ekf:=false`로 실행.
  - FAST-LIO-Localization + Nav2 실행.

## 문서

- README를 FAST-LIO 기반 현행 구조로 갱신.
- `OPERATION_GUIDE.md`를 매핑 -> pcd2pgm -> 측위 검증 -> Nav2 주행 순서로 갱신.
- `JETSON_SETUP.md`를 UDP 직접 파싱 기준 설치 절차로 갱신.
- `SETUP_CHECKLIST.md`를 실차 확인값 중심으로 갱신.
- `TODO.md`에 2026-07-10 기준 남은 작업 정리.

## 검증/실험 기록

- `colcon build --cmake-args -DBUILD_TESTING=OFF` 통과 기록 있음.
- FAST-LIO2 매핑으로 `alm_3d_map.pcd` 생성.
- `pcd2pgm.py`로 `alm_map.pgm/yaml` 생성.
- FAST-LIO-Localization 측위 성공 기록 있음.
- `map_publisher.py`와 `localization.rviz`로 2D 트래킹 뷰 검증.

## 현재 남은 정리 포인트

- `localization.launch.py`의 `map_pcd`와 `fastlio_relocalization.yaml`의
  `prior_map_path` 동기화.
- ICP voxel leaf size 튜닝.
- Python UDP point parser CPU 부하 개선.
- 실차 본체 연결 후 Nav2 goal -> `/cmd_vel` -> `/mcu/command` -> 실제 주행 검증.
- livox_ros_driver2 런타임 미사용과 빌드 의존성의 경계가 헷갈리지 않도록 추가 정리.
