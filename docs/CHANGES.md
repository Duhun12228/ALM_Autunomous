# 작업 내역 정리 — 빈 워크스페이스 → 자율주행 스택

원래 상태: 7개 패키지의 **뼈대(package.xml / CMakeLists / 빈 폴더)** 와 커스텀 메시지 2개,
그리고 `alm_sensors` 일부(Livox UDP 파서·EBIMU IMU·pointcloud_to_scan)만 있었습니다.
아래는 그 위에 **추가(＋)** 하거나 **수정(✎)** 한 전체 내역입니다.

범례: ＋ 새 파일 / ✎ 기존 파일 수정

---

## alm_msgs (Jetson↔STM32 메시지)
- ✎ `msg/McuState.msg` — STM32 업링크에 **per-wheel 엔코더 피드백** 추가
  - `float32[2] steer_angle` (앞축/뒤축 조향각), `float32[4] wheel_speed` (4구동 각속도)
  - (McuCommand는 기존 그대로: cmd_vel + drive_mode + enable_motors + emergency_stop)

## alm_description (URDF/TF)
- ＋ `urdf/alm_robot.urdf.xacro` — **4WIS 로봇 URDF**. CAD 실측(바퀴반지름 0.103, 앞축 x+0.611/뒤축 x−0.301, 트랙 ±0.5, 차체 1.2×0.9872×0.45, 80kg). base_link + 4×(조향링크+바퀴링크). 라이다 프레임은 위치 미확정이라 제외
- ＋ `launch/description.launch.py` — robot_state_publisher(URDF→TF). `standalone:=true`면 joint_state_publisher_gui, `rviz:=true`면 RViz
- ＋ `rviz/alm.rviz` — RViz 기본 설정(로봇/TF/scan/map/plan)
- ✎ `CMakeLists.txt` — 설치 대상 `meshes`(없음) → `launch` 로 교체
- ✎ `package.xml` — xacro, robot_state_publisher, joint_state_publisher_gui, rviz2 의존성 추가

## alm_sensors (센서 — 공식 Livox 드라이버로 전환)
- ＋ `config/MID360_config.json` — Livox MID-360 네트워크 설정(호스트/라이다 IP, 포트)
- ＋ `scripts/imu_relay.py` — **내장 6축 IMU** `/livox/imu` → `/imu/data` 재발행. orientation 무효화(EKF가 가짜 방향 융합 방지)
- ✎ `launch/lidar.launch.py` — 기존 UDP 파서 → **공식 livox_ros_driver2** 노드로 교체. + pointcloud_to_scan(/scan) + base_link→livox_frame static TF(라이다 위치 파라미터)
- ✎ `launch/imu.launch.py` — 기존 EBIMU 퍼블리셔 → imu_relay 실행으로 교체
- ✎ `config/sensors.yaml` — 토픽/프레임을 Livox 기준으로 갱신
- ✎ `CMakeLists.txt` — imu_relay.py 설치 추가
- ✎ `package.xml` — livox_ros_driver2, launch_ros 의존성 추가
- (기존 `livox_udp_pointcloud2.py`, `imu_publisher.py`, `pointcloud_to_scan.py`는 유지. 앞 2개는 미사용)

## alm_navigation (EKF / SLAM / AMCL·Nav2) — 전부 신규
- ＋ `config/ekf.yaml` — robot_localization EKF. `/wheel_odom` + `/imu/data` 융합 → `/odometry/filtered`, TF odom→base_link (2D 모드)
- ＋ `config/slam_toolbox.yaml` — slam_toolbox 매핑. 입력 **2D /scan**, base_frame base_link, use_sim_time false
- ＋ `config/nav2.yaml` — Nav2 전체. AMCL / planner(A*) / controller(DWB) / costmap. **footprint 실측 사각형**, 회피 소스 **2D /scan + 3D /livox/lidar 둘 다**, use_sim_time false
- ＋ `launch/ekf.launch.py` — EKF 노드 실행
- ＋ `launch/slam.launch.py` — slam_toolbox 실행
- ＋ `launch/navigation.launch.py` — nav2_bringup(AMCL+Nav2)을 우리 파라미터로 실행
- ＋ `maps/.gitkeep` — 맵 저장 폴더
- ✎ `package.xml` — robot_localization, slam_toolbox, nav2_bringup 등 의존성 추가

## alm_base_control (모드 선택 + 안전 게이팅) — 전부 신규
- ＋ `scripts/command_manager.py` — **핵심 노드**. `/cmd_vel`+`/drive_mode`+`/emergency_stop` → auto의 **normal/spin/crab 자동선택**(참고 레포 로직 포팅) + 속도/가속 제한 + timeout/e-stop → `/mcu/command`
- ＋ `config/base_control.yaml` — 속도한계, auto 모드 임계값(spin 진입 |wz|≥0.35 등)
- ＋ `launch/base_control.launch.py`
- ✎ `CMakeLists.txt` — command_manager.py 설치 추가
- ✎ `package.xml` — rclpy 등 추가

## alm_mcu_interface (Jetson↔STM32 UART 브리지) — 전부 신규
- ＋ `scripts/mcu_bridge.py` — UART 송수신(CRC16 프레이밍). `/mcu/command`→STM32, STM32→`/mcu/state`+`/wheel_odom`+`/joint_states`. **기구학은 안 함**(STM32 담당)
- ＋ `config/mcu_interface.yaml` — 포트(/dev/ttyTHS1), baud, 토픽/프레임
- ＋ `docs/uart_protocol.md` — **STM32 팀용 프로토콜 규격서**(프레임/CRC/바이트 레이아웃/역·정기구학 가이드)
- ＋ `launch/mcu_interface.launch.py`
- ✎ `CMakeLists.txt` — mcu_bridge.py + docs 설치 추가
- ✎ `package.xml` — rclpy, python3-serial 추가

## alm_bringup (최상위 통합 launch) — 전부 신규
- ＋ `launch/robot.launch.py` — 상시 스택(description+sensors+ekf+base_control+mcu_interface)
- ＋ `launch/slam.launch.py` — robot + slam_toolbox (매핑)
- ＋ `launch/navigation.launch.py` — robot + AMCL + Nav2 (자율주행)
- ＋ `config/.gitkeep`
- ✎ `package.xml` — alm_* 하위 패키지 의존성 추가

## 문서 (리포 루트)
- ✎ `README.md` — 파이프라인/패키지/설치/실행/모드 설명으로 대폭 갱신
- ＋ `SETUP_CHECKLIST.md` — 실차 전 확인/수정할 값(지오메트리, 라이다 위치, STM32 등)
- ＋ `docs/OPERATION_GUIDE.md` — 매핑→저장→자율주행 운영 순서
- ＋ `docs/CHANGES.md` — (이 문서)

---

## 검증 완료
- `colcon build` 7개 패키지 전부 통과
- xacro 파싱 OK(9 links / 8 joints), McuState 배열 필드 정상
- UART payload 크기 18/63 bytes = 프로토콜 문서와 일치
- command_manager 실행 → auto에서 회전 시 spin, 전진 시 normal 전환 확인

## 리뷰 후 개선 (2026-07-09)
- **[1] `/scan` 변환을 C++ 표준으로 교체**: `lidar.launch.py` 가 파이썬 `pointcloud_to_scan` 대신
  `pointcloud_to_laserscan`(C++) 노드 사용. `target_frame=base_link` 라 높이필터가 '지면 기준' →
  바닥 오탐↓, Jetson 부하↓. (`sudo apt install ros-humble-pointcloud-to-laserscan`)
- **[7] MCU fault 반영**: command_manager 가 `/mcu/state` 구독, fault/estop 보고 시 즉시 정지.
- **[8] 오도메트리 워치독**: 주행 중 `/odometry/filtered` 가 `odom_watchdog_sec`(0.5s) 넘게 끊기면 정지.
  (odom 을 한 번 받은 뒤 끊기는 경우만 → 부팅 시 오탐 방지)
- **[9] rate-limit 옵션화**: `enable_rate_limit`(기본 true). velocity_smoother 와 이중이면 false 가능.
- 검증: MCU fault→정지(enable_motors=false, emergency_stop=true) 실동작 확인.

## 내가 만들지 않은 것 (기존 패키지 재사용)
SLAM(slam_toolbox), AMCL·planner·controller(nav2), EKF(robot_localization),
LiDAR 드라이버(livox_ros_driver2) — 이들은 **직접 코딩이 아니라 설정+launch로 구동**.
우리 코드는 그 사이 접착제(센서 relay, EKF 설정, 모드 매니저, UART 브리지)입니다.
