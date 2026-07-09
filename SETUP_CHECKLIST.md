# 셋업 체크리스트 — 실차 적용 전 확인/수정할 값

내가 코드를 짤 때 CAD 실측값 또는 합리적 기본값으로 채워둔 항목들입니다.
실제 하드웨어에 맞게 확인·수정하세요. (##TODO## / ##CONFIRM## 주석으로도 표시해둠)

## 1. 로봇 지오메트리 — `alm_description/urdf/alm_robot.urdf.xacro`
CAD(alm_p1_ASSEM_0526) 실측으로 채움. 값이 맞는지만 확인:
- [ ] `wheel_radius=0.103`, `wheel_width=0.0488`
- [ ] 휠 위치 `front_x=+0.6106`, `rear_x=-0.3010`, `half_track=0.500`
      (앞/뒤축이 비대칭임 — CAD 상 차체중심 기준. base_link=차체 XY 중심)
- [ ] 차체 `1.200 × 0.9872 × 0.450`, 질량 80 kg
- [ ] 조향 한계 toe-in 90° / toe-out 45° (URDF 조인트는 시각화용 ±90°)
> 이 값을 바꾸면 아래 Nav2 footprint 도 함께 맞추세요.

## 2. Nav2 footprint / 속도 — `alm_navigation/config/nav2.yaml`
- [ ] `footprint: [[0.72,0.52],[0.72,-0.52],[-0.65,-0.52],[-0.65,0.52]]` (외곽 사각형, URDF 와 일치)
- [ ] 최고속도 `max_vel_x`(0.45), `max_vel_theta`(0.8), `velocity_smoother` 값 — 실제 플랫폼 사양으로
- [ ] `robot_model_type`: 현재 `DifferentialMotionModel` (crab 상시 사용 시 `OmniMotionModel` 로)
- [ ] `min_obstacle_height/max_obstacle_height`(pointcloud costmap) — 라이다 마운트 높이 기준

## 3. LiDAR — `alm_sensors`
- [ ] `config/MID360_config.json`: lidar `ip`(예 192.168.1.12), host ip(192.168.1.5) — 실제 IP
- [ ] **LiDAR 마운트 위치** (미확정): `lidar.launch.py` 의 `lidar_x/lidar_y/lidar_z`
      (base_link→livox_frame static TF). 기본 `0,0,0.5`
- [ ] `pointcloud_to_scan` 의 `min_height/max_height`(라이다 기준 상대높이) — 마운트 높이 맞춰 조정
- [ ] `livox_udp_pointcloud2.py` 의 `HOST_IP/POINT_PORT` 상수 — 실제 네트워크와 일치하는지
- [ ] 런타임은 UDP 직접 파싱 사용. livox_ros_driver2 노드는 기본 launch 에서 실행하지 않음
- [ ] 단 전체 워크스페이스 빌드 시 FAST-LIO/ICP 의 메시지 헤더 의존성 때문에 Livox-SDK2/livox_ros_driver2 빌드 환경 확인

## 4. IMU
- [ ] Livox 내장 6축 IMU 사용 (`/livox/imu` → `/imu/data`). orientation 없음 → EKF 는 yaw rate + 선가속도만 융합
- [ ] (구 EBIMU 스크립트 `imu_publisher.py` 는 남겨뒀지만 미사용)

## 5. STM32 / UART — `alm_mcu_interface`
- [ ] 포트 `/dev/ttyTHS1` (확인됨), baud `115200` — STM32 설정과 일치하는지 ##CONFIRM##
      (`sudo chmod 666 /dev/ttyTHS1` 또는 dialout 그룹)
- [ ] **STM32 펌웨어**를 `docs/uart_protocol.md` 규격대로 구현 (프레임/CRC16/역·정기구학)
      - Command 18 bytes, State 63 bytes, CRC16-CCITT
      - 조향 구조: 현재 **2축 조향(steer_angle[2]=front/rear) + 4구동(wheel_speed[4])** 가정.
        실제 4바퀴 독립조향이면 `McuState.msg` 의 `steer_angle` 길이 4 로 늘리고
        `mcu_bridge.py` 의 `STATE_FMT`/`_handle_state` 및 프로토콜 문서 함께 수정.
- [ ] STM32 명령 timeout(권장 200 ms) 시 정지 로직

## 6. base_control 안전값 — `alm_base_control/config/base_control.yaml`
- [ ] 속도/가속 제한(`max_linear_x` 등) 실제 플랫폼에 맞게
- [ ] `default_drive_mode`(auto), `cmd_timeout_sec`(0.5) 확인
- [ ] `/emergency_stop`(std_msgs/Bool) 배선 — 물리 e-stop 을 이 토픽으로 연결

## 7. 최초 브링업 검증 순서(권장)
1. `robot.launch.py` → `ros2 topic list` 로 `/livox/lidar /livox/imu /scan /imu/data /wheel_odom` 확인
2. `ros2 run tf2_tools view_frames` 로 TF 트리(map↔odom↔base_link↔livox_frame) 확인
3. teleop 으로 `/cmd_vel` 발행 → `/mcu/command` 나오는지, STM32 가 움직이는지
4. `slam.launch.py` 로 FAST-LIO2 3D 맵 작성 → `/map_save`
5. `pcd2pgm.py` 로 2D `alm_map.pgm/yaml` 생성
6. `localization.launch.py` 로 FAST-LIO-Localization 측위 검증
7. `navigation.launch.py map:=... map_pcd:=...` 로 자율주행

## 8. 현재 코드 기준 주의
- [ ] 주행 모드에서는 EKF 를 끕니다. `navigation.launch.py` 가 `robot.launch.py use_ekf:=false` 로 실행합니다.
- [ ] Nav2 의 odom topic 은 `/Odometry` 입니다. FAST-LIO-Localization 이 발행합니다.
- [ ] `localization.launch.py map_pcd:=...` 와 `fastlio_relocalization.yaml` 의 `prior_map_path` 는 같은 PCD 를 가리켜야 합니다.
