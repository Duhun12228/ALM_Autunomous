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

## 출발 헤딩 정렬 — 경로는 항상 내 헤딩에 접해서 나온다 (2026-08-25)

**증상**: 벽을 보고 세워 둔 로봇 뒤쪽에 목표를 찍으면 전역경로가 출발하자마자
옆으로 크게 튐.

**원인**: 경로는 정상이었다. Hybrid-A\* 는 시작 상태 θ 를 현재 로봇 헤딩으로 두고
R_min 원호만 이어 붙이므로 **경로는 언제나 현재 헤딩에 접해서 출발**한다. 고쳐야 할
것은 경로가 아니라 출발 헤딩인데, 그걸 담당하는 ALIGN 이 출발 시점에 작동하지 않고
있었다 — 경로 헤딩오차의 상한이 `lookahead / R_min = 1.0/1.643 = 34.9°` 인데
진입 문턱이 60° 라 **수학적으로 도달 불가능**했다.

**고친 것 (세 겹)**

- `align_lookahead_m_stopped: 3.0` — 출발 구간 전용 lookahead. 오차 상한을
  34.9° → 104.6° 로 연다. 주행 중 값(1.0)은 그대로 — 실측 헤딩오차 분포
  (p90 39.8°)가 전부 그 기준이라 늘리면 정상 선회를 오인한다.
- `align_goal_bearing_*` — **출발 전 목표 방위각 pre-align.** 경로 대신 목표까지의
  직선 방위각을 오차로 쓴다. lookahead 확장으로도 '접선 출발' 이라는 사실 자체는
  못 바꾸므로, 180° 를 즉시 보는 것은 이것뿐이다. 목표당 1회, 아직 0.30 m 미만
  이동한 동안만. 목표 좌표는 `/plan` 끝점에서 뽑는다(`/goal_pose` 를 안 쓰는 이유:
  목표가 NavigateToPose 액션으로 직접 올 수도 있다).
- `align_relatch_stopped: true` — 출발 구간 목표 재래치. 한 기동이 도는 각도가
  '래치각 − 이탈각' 으로 묶이던 것을 푼다. 폭주 방지는 **회전 방향 고정**(부호가
  뒤집히면 중단)과 **늘리기만** 두 겹.

**그 과정에서 잡은 것**

- `align_enter_deg_stopped` 25° → 60°. lookahead 가 3 m 가 되면 25° 는 정상적인
  S자 출발에도 매번 걸려 불필요한 spin 을 낸다(시뮬에서 '비스듬 30° 목표' 가
  spin 1회 → 0회로 개선됐다).
- **'정지' 를 속도가 아니라 이동거리로 정의**(`align_start_travel_m: 0.30`).
  `|vx| < 0.05` 로 두면 목표 수락 후 0.15 s 만에 조건이 무너지는데 진입 지속시간이
  바로 그 0.15 s 라 **창이 사실상 0** 이었다(통합 시험에서 한 번도 안 걸림).
  이 정의가 '복귀 dwell 중 ALIGN 재진입 루프' 도 함께 막는다.
- `AlignManeuver.update()` 에 진입 문턱 오버라이드 추가. 없으면 방위각(40°)을
  넣어도 `enter_deg_stopped`(60°)가 실효 문턱이 되어 조용히 무시된다.
- `nav2_kinematic_check.py`: 들여쓰기가 무너져 `align_enabled: false` 에서
  `UnboundLocalError` 로 죽던 것 수정. 출발 시 헤딩오차 상한 검사 신규 추가.
- `run_recorder.py`: spin 구간 길이에 전체 누적값을 넣던 버그 수정. 구간별 실제
  회전각을 함께 기록하고, 'spin 구간 4회 이상' 소견을 추가(재래치가 안 먹는 신호).
- `alm_navigation/package.xml`: `alm_msgs`·`sensor_msgs_py`·`rclpy`·numpy 등
  파이썬 노드가 직접 import 하는 의존 9개 누락 보완.

**검증**: `command_manager._tick` 을 ROS 스텁 위에서 그대로 돌리는 통합 시험
(가짜 Nav2 = Dubins 유사 경로 2.5 Hz 재계획 + Ackermann 한계 추종), 목표 8 m.

| 목표 방향 | 이전 | 현재 |
|---|---|---|
| 정면 0° | spin 0회 / 50.1 s | spin 0회 / 50.1 s (회귀 없음) |
| 비스듬 30° | spin 1회 / 61.6 s | spin **0회** / 50.5 s |
| 옆 90° | spin 4회 / 97.0 s | spin **1회** / 65.8 s |
| 뒤 180° | spin **8회** / 145.7 s | spin **1회** / **72.3 s** |

실차 검증은 남음. `run_recorder` 의 `mode.spin_segments` 가 4개 이상이면 재래치가
안 먹고 있는 것이다.

## 전수 조사 반영 — 매핑 파이프라인 · 제어 · 진단지표 (2026-08-25)

`5ded55f` 기준 전수 조사에서 나온 항목을 반영했다. 하드코딩 절대경로
(`voice.yaml` 등)는 로봇 쪽 실제 경로라 **그대로 두었다.**

### 재매핑 전에 고쳐야 했던 것 (안 고치면 결과가 그대로 굳는다)

- **웹으로 구우면 지면 기준 밴드가 한 번도 안 걸리고 있었다.** `api.py` 가
  `--z-min/--z-max` 를 **무조건** 넘겼는데, `pcd2pgm` 은 이 인자가 하나라도
  오면 절대 z(호환) 모드로 떨어진다. 즉 지면 자동추정이 웹 경로에서 죽어
  있었다. `-0.3` 의 정당성은 '라이다 마운트 0.5 m' 가정에 전부 걸려 있고 그
  TF 는 아직 추정값이다 — 마운트가 0.7 m 면 밴드 하한이 지면 위 0.4 m 가 되어
  그보다 낮은 턱·박스가 **가짜 자유공간**이 된다. 미관측보다 나쁜 종류다.
  · 백엔드: 요청에 명시적으로 있을 때만 절대 z 를 넘기고, 기본은
    `--obstacle-min-h/--obstacle-max-h`(지면 기준). 절대 z 를 쓰면 경고 로그.
  · UI: 지면 기준 필드를 기본으로, 절대 z 는 `<details>` 안으로 옮기고
    **비어 있으면 아예 안 보낸다.** 예전엔 -0.3/1.5 를 하드코딩해 항상 보냈다.
- **`--min-points` 1 -> 2** (CLI · 웹 모두). 밴드가 지면 위 0.15~1.80 m 로
  넓어져 밴드에 드는 점이 크게 늘었는데 1 이면 반사 노이즈 한 점이 점유 셀이
  된다. 레이캐스팅은 `free &= ~occupied` 라 광선이 지나가도 안 지워진다.
- **`scan_recorder` 의 중간 저장이 콜백을 막고 있었다.** `_on_cloud` 안에서
  30 s 마다 누적 전체를 `savez_compressed` 로 재압축했다. 후반 수십 MB 구간에서
  수 초씩 걸리고, 그동안 `/cloud_registered` 큐(depth 20)가 넘친다.
  **유실된 스캔은 레이캐스팅에 구멍을 내고 그 자리는 미관측으로 남는데 로그에는
  아무것도 안 남는다.** 이제 스냅샷만 뜨고 별도 스레드에서, 중간 저장은
  무압축(`savez`)으로 쓴다. 최종 저장만 압축한다. 버린 스캔 수도 종료 시 경고.
- **`map_manager` 가 격자를 직접 측정한다.** mtime 으로 추측하지 않고 grid.pgm 의
  점유/자유/미관측 비율을 센다(`bytes.count`, numpy 없이 수 ms). 미관측이 50% 를
  넘으면 stale + "투영 방식으로 구운 격자" 로 표시하고, `scans.npz` 유무·신선도,
  `grid.yaml` 의 `free_thresh` 오설정(0.196 초과)도 함께 본다.
  자산 종류를 늘리지 않은 이유: `MapAsset.KIND_*` 는 메시지 상수라 웹 UI 까지
  번진다. scans 의 존재 이유가 '이 격자를 레이캐스팅으로 굽는 것' 하나뿐이라
  격자의 속성으로 다루는 편이 맞다.
- **`pcd2pgm` 이 구역별 지면 편차를 보고한다.** 전역 단일 평면 가정이 성립하는지
  4x4 구역으로 나눠 재고, 0.20 m 를 넘으면 경고한다(합성 시험: 평평 0.02 m,
  3% 경사 0.91 m). 판정하지 않고 보고만 한다 — 지면 모델을 고치는 건 별개다.

### 제어

- **`velocity_smoother` 각가속 wz ±1.5 -> ±0.8.** `base_control.max_accel_theta`
  와 `behavior_server.rotational_acc_lim` 은 0.8 로 내렸는데 여기만 남아 있었다.
  더 빡빡한 0.8 이 뒤에서 이기므로 거동은 같았고 그래서 아무도 못 봤다 —
  `nav2_kinematic_check.py` 가 각가속을 아예 안 보고 있었기 때문이다.
- **`align_exit_deg` 15.0 -> 10.0.** `yaw_goal_tolerance` 가 0.20 rad = 11.46°
  인데 ALIGN 이 15° 에서 손을 떼면 그 사이 3.5° 를 맞출 주체가 없다. MPPI 는
  Ackermann 이라 제자리 회전을 못 내고, ALIGN 은 진입 문턱 아래에서 재진입하지
  않는다. 이제 목표 체커 허용치 **안쪽**에서 넘긴다.
- **`direct_*` 4개를 yaml 로 승격** (`direct_topic`, `direct_timeout_sec`,
  `direct_rpm_accel`, `direct_rpm_decel`). 코드 기본값으로만 동작해 웹 수동주행의
  가감속을 조정할 수 없었다. 선언 81개 = yaml 81개로 맞췄다(사문화된 키 0개).

### 진단 지표 — 안 고치면 실차 데이터를 못 믿는다

- **`clamp_frac` -> `infeasible_frac` 으로 교체.** 예전 지표는 `/cmd_vel` 요청과
  `out.cmd_vel` 실제의 gap 을 봤는데, 그 gap 에는 조향 클램프뿐 아니라
  `max_accel_theta` 램프가 섞인다 — 가감속 구간이 전부 '조향 한계로 잘림' 으로
  계수됐다. 하필 이게 와리가리 판정의 핵심 지표다. 이제 Nav2 가 낸 (vx, wz)
  **쌍 자체**가 `|wz| <= |vx|/R_min` 을 지키는지 본다. 우리 램프와 무관하고,
  `command_manager` 의 클램프 식과 같다. normal 모드에서만 판정한다.
- **`stopped_frac` 의 분모가 노드 수명 -> 목표 수행 시간.** 목표를 주기 전
  대기시간이 그대로 '정지' 로 쌓여 어떤 주행이든 30% 소견이 떴다.
- **주행 기록 회전 삭제** (`keep_runs`, 기본 50). 기록은 계속 켜 둔다 — 단일
  실행 A/B 는 믿을 수 없어 여러 판을 모아야 하고 거기에 이 노드의 가치가 있다.
  이름이 정확히 `run_<날짜>_<시각>` 인 디렉터리만 지운다.
- **`nav2_kinematic_check.py` 검사 2종 추가**: 각가속 3곳 일치,
  `align_exit_deg <= yaw_goal_tolerance`.

### 위생

- `.gitignore`: `maps/**/*.npz` 주석이 `sc_db_*` 얘기라 지금 뭘 왜 빼는지 안
  읽혔다. **맵 폴더를 다른 장비로 옮길 때 scans.npz 를 반드시 같이 가져가라**는
  경고를 규칙 옆에 붙였다 — 빠뜨리면 받는 쪽에서 미관측 8할이 재현되고
  cloud.pcd 만으로는 소급 생성이 안 된다.
- `docs/TODO.md`: `free_thresh` 항목 닫음. **다만 그 항목이 적어둔 목표값
  0.196 자체가 틀렸다** — 205 의 실제 occ 가 0.19608 이라 8e-5 차이로 경계에
  걸린다. 채택값은 0.19.
- `docs/TODO.md`: **URDF vs CONS 항목은 열어 뒀다.** 검사기가 초록불인 이유는
  URDF 를 CONS 에 맞춰 고쳤기 때문이지 실측으로 확인했기 때문이 아니다.
  `##CONFIRM## front_x / rear_x 배분` 이 그대로 남아 있다 — 휠베이스 **합**만
  맞췄고 앞뒤 배분은 미확인이며, 그건 R_min 이 아니라 선회 중 차체 궤적에
  들어간다. 검사기 초록불로 닫으면 살아있는 질문이 사라진다.
- `alm-webui-v0.6/BUTTON_COMMAND_MAP.md`: 2D 맵 변환 표를 새 필드로 갱신.

### 검증

- `nav2_kinematic_check.py` 전부 일치 (exit 0)
- `path_align.py` / `fourwis_encode.py` 자체 시험 전 항목 통과
- 격자 측정: cschool 미관측 87.91% · alm_lab 81.26% — 독립 계산과 일치
- 지면 편차: 평평한 합성 지면 0.02 m vs 3% 경사 0.91 m (문턱 0.20 m 로 분리됨)
- ALIGN 통합 회귀 (ROS 스텁, 목표 8 m):

  | 목표 방향 | 이전 | 현재 |
  |---|---|---|
  | 정면 0° | spin 0회 / 50.1 s | spin 0회 / 50.1 s |
  | 비스듬 30° | spin 1회 / 62.1 s | spin 0회 / 50.5 s |
  | 옆 90° | spin 3회 / 87.6 s | spin 1회 / 66.4 s |
  | 뒤 180° | spin 6회 / 125.9 s | spin 1회 / 72.8 s |

