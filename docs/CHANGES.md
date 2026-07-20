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
- AIST 공개 MID-360 ROS 2 bag(약 35초 구간)으로 SC-LIO-SAM 통합 검증:
  - driver2 `timestamp`/`line`을 상대 `time`/`ring`으로 변환.
  - bag IMU의 g 단위를 `9.80665` 배율로 m/s² 변환.
  - deskew 약 7.1 Hz, mapping odometry 약 4.2 Hz, velocity reset/TF 오류 0회.
  - 약 75 m 이동 구간에서 최종 z 약 -1.38 m로, 수정 전 수백 m 수직 발산 해소.
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

## 방식 B: Scan Context 초기위치 자동화 (dev/fastlio2-sc, 2026-07-14)

RViz "2D Pose Estimate" 수동 초기화를 대체하는 글로벌 재측위. vendored
C++(icp_relocalization/fast_lio)는 무수정 — `icp_node` 가 이미 `/initialpose` 를
구독하므로 그 앞단에 SC 노드만 추가했다. 전부 `alm_navigation` 파이썬.

- `scripts/scan_context.py`: SC 디스크립터(극좌표 ring×sector, bin=max z)와
  매칭(ring key 후보 → 전 shift 코사인 거리, 최적 shift=yaw) 공용 모듈.
  - 열 한쪽만 점유 시 거리 1 페널티: 이것 없으면 거의 빈 디스크립터가
    아무 스캔과도 거리 0 으로 오매칭됨 (selftest 3/20 → 20/20 의 핵심 수정).
- `scripts/sc_build_db.py`: prior map.pcd → 격자(기본 0.5 m) 가상 키프레임 SC
  DB(.npz). 유효성: 점수/장애물내부(clearance)/방위 커버리지. `--selftest N` 으로
  가상스캔 자가검증. **z밴드(기본 [-0.3, 1.0])는 반드시 천장 아래** — 천장이
  들어가면 모든 bin 이 천장 높이로 균일해져 장소 구분이 무너진다 (실측 확인).
- `scripts/sc_localizer.py`: /livox/lidar 10프레임 누적(정지 상태) → SC 매칭 →
  상위 후보를 `/initialpose` 로 순차 발행(후보당 ICP 12 s 대기, 실패 시 재스캔
  루프) → `/icp_result` 수신 시 종료. 디버그용 `/sc_candidates`(PoseArray).
- `localization.launch.py`: `auto_init`(기본 true)·`sc_db` 인자 추가.
- 검증(집 맵 764k점, LiDAR 미연결 오프라인):
  - sc_build_db selftest 30/30 (pos 중앙값 0.18 m, yaw 1.1°).
  - 합성스캔 E2E(맵에서 뜬 가상 스캔 → sc_localizer → icp_node): 2개 pose 모두
    첫 후보에서 ICP 수렴, `/icp_result` 오차 ~0.3 m / 3°.
  - 실센서/실차 검증은 남음 (누적 스캔은 맵과 달리 가림(occlusion) 있음).

## 방식 C: SC-LIO-SAM 매핑 (dev/sc-lio-sam, 2026-07-14)

FAST-LIO2 매핑을 **SC-LIO-SAM**(LIO-SAM + Scan Context 루프클로저)으로 교체하는
브랜치. 넓은 공간(창고/긴 복도/순환 경로)에서 루프클로저로 누적 드리프트를
보정해 맵 품질을 높이는 것이 목적. 방식 B(SC 재측위)가 머지되어 있어 만든 맵으로
재측위 자동화까지 동일하게 쓴다.

- **GTSAM 4.1.1 ARM 소스빌드** (`ALM_auto_ws/thirdparty/src/gtsam` →
  `thirdparty/install`): 과거 FAST-LIO-SAM 시도를 접게 했던 난관.
  `-DGTSAM_USE_SYSTEM_EIGEN=ON`(PCL/ROS 와 Eigen 정합, alignment 크래시 방지)
  `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF` 로 빌드 성공. colcon 이 gtsam 의
  package.xml 을 워크스페이스 패키지로 오인하므로 `thirdparty/COLCON_IGNORE` 필요
  (gitignore 영역이라 깨끗한 체크아웃에선 재생성해야 함 — JETSON_SETUP 참고).
- **vendored `src/thirdparty/SC-LIO-SAM`** (패키지명 lio_sam): TixiaoShan/LIO-SAM
  ros2 브랜치 기반 + gisbi-kim SC-LIO-SAM(ROS1)의 SC 루프클로저를 diff 로 추출해
  이식. 이식 내용: SCManager + 키프레임마다 SC 디스크립터(SINGLE_SCAN_FULL,
  deskewed raw 0.5 복셀), performSCLoopClosure(SC 검출→base_key 기준 ICP 검증→
  robust Cauchy 루프팩터), loopFindNearKeyframesWithRespectTo, multimap/
  SharedNoiseModel 자료구조 변경. giseop 의 SCD/g2o 파일 덤프는 제외(인메모리만).
- **MID-360 적응** (imageProjection ALM 패치): UDP 파서 출력(x,y,z,intensity,time,
  ring 없음)을 LivoxPointXYZIT 로 받고 고도각(-8~55도)을 N_SCAN(16) 밴드로 양자화해
  ring 합성. column 은 비반복 스캔에 맞는 도착순(LIVOX 경로) 사용.
- **6축 IMU 대응**: LIO-SAM 은 orientation 필요(deskew 초기 roll/pitch) →
  `alm_sensors/imu_orientation.py`(Madgwick, 자력계 없음)로 /livox/imu →
  /livox/imu_orient 합성. yaw 는 드리프트하므로 useImuHeadingInitialization=false.
- **Scancontext 실내 튜닝**: PC_MAX_RADIUS 80→40, LIDAR_HEIGHT 2.0→0.5,
  PC_MAX_Z=3.5 천장컷 추가(방식 B 에서 실측한 천장 균일화 문제 예방).
- `slam_sc.launch.py`(4노드+IMU필터+static map→odom) + `config/sc_lio_sam.yaml`.
  맵 저장: `/lio_sam/save_map` 서비스 → maps/sc_lio_sam/GlobalMap.pcd.
- **빌드/런타임 마무리**: 사용하지 않는 `gtsam_unstable` 헤더 제거, GTSAM/Eigen
  `Vector` 충돌 해소, Scan Context 각도 계산을 `atan2`로 안전화. GTSAM의 간접
  의존성 `libmetis-gtsam.so`를 찾도록 ament 환경 훅 추가. IMU 실행 파일 내부의
  두 노드 이름이 launch remap으로 겹치던 문제와 static TF 구식 인자도 정리.
- 검증: `BUILD_TESTING=OFF` 전체 워크스페이스 11패키지 빌드 통과. `slam_sc.launch.py`
  6프로세스(IMU 필터, static TF, LIO-SAM 4노드) 기동 스모크 통과.
  **루프클로저/맵품질/Orin 부하는 실센서 데이터 필요** (다음 단계).
