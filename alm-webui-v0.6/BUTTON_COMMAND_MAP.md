# WebUI 버튼 → Jetson 실행 명령 매핑

WebUI의 모든 조작 요소가 연동 후 **Jetson에서 실제로 무엇을 실행하는지**를 버튼 단위로 정리한 문서입니다. [`JETSON_INTEGRATION_PLAN.md`](JETSON_INTEGRATION_PLAN.md)가 "무엇을 만들어야 하나"라면, 이 문서는 "각 버튼이 눌리면 어떤 명령이 나가나"입니다.

기준: `ALM_auto_ws/src` 실물 (2026-08-10). 표의 명령은 **backend가 rclpy로 수행할 내용을 CLI 형태로 표기**한 것입니다 — 실제로는 `ros2` 명령을 subprocess로 부르는 게 아니라 동등한 API 호출을 합니다. 단, `pcd2pgm.py`/`fpfh_map_builder`와 `ros2 launch`만은 진짜 subprocess 실행입니다.

## 구현 상태 (2026-08-10 · Phase 3 완료)

| 그룹 | 상태 | 비고 |
|---|---|---|
| 전역 E-STOP · 해제 | ✅ **실동작** | `command_manager` 래치 + `/emergency_stop/release` 서비스 |
| 웹 세션 제어권 | ✅ **실동작** | `alm_web_backend` 리스 락 (TTL 15초) |
| 매핑 시작·종료·저장 | ✅ **실동작** | launch 슬롯 + `/map_save` |
| pcd2pgm · FPFH DB | ✅ **실동작** | subprocess + 실 stdout 스트림 |
| 맵 생성 · 활성 전환 | ✅ **실동작** | `maps/<이름>/manifest.yaml`, `active.yaml` |
| 맵 자산 조회 | ✅ **실동작** | `/alm/map_inventory` (backend 불필요) |
| 측위 (`#autoLocalization` 등) | ⏳ 목업 | Phase 4 |
| 자율주행 (`#startNavigation` 등) | ⏳ 목업 | Phase 4 — Nav2 기동 검증 선행 |
| 수동주행 (`#enterManual` 등) | 🚫 **보류** | `base_control.yaml` 의 4WIS 상수 8개가 `##CONFIRM##` 미확정. 확정 전에는 twist 가 그대로 엉뚱한 rpm/조향각이 된다 |

**API 표면**: `GET /api/health` · `/api/limits` · `/api/session*` · `POST /api/estop`(락 예외) · `/api/estop/release` · `/api/mapping/{start,stop,save}` · `/api/jobs/{pcd2pgm,fpfh}` · `GET /api/jobs/<id>[/stream]` · `POST /api/maps` · `PUT /api/maps/active`

모든 요청에 `Authorization: Bearer $ALM_WEB_TOKEN`. 상태를 바꾸는 요청은 추가로 `X-ALM-Session` 이 현재 제어권 보유자와 일치해야 합니다. **E-STOP 만 락 예외** — 남이 조작 중이라고 로봇을 못 세우면 그게 사고입니다.

---

## 0. 경로 분류

| 기호 | 경로 | 의미 |
|---|---|---|
| **L** | 로컬 | 브라우저 안에서 끝남. Jetson으로 아무것도 안 나감 |
| **B** | → `alm_web_backend` (HTTPS/WS) | 명령. backend가 ROS2 서비스/액션/토픽/프로세스로 번역 |
| **F** | ← `foxglove_bridge` (WSS) | 읽기 전용 구독. 버튼이 아니라 화면 갱신 |

**원칙: 브라우저가 Foxglove Bridge로 직접 퍼블리시하는 경로는 만들지 않습니다.** 모든 쓰기는 B를 거칩니다 (§8 allowlist 참조).

---

## 1. 전역 — HUD 리본 · 안전 스트립

| 요소 (id) | 경로 | Jetson에서 실행되는 것 | 비고 |
|---|---|---|---|
| **E-STOP** `#globalEstop`<br>(0.9초 홀드) | **B** ✅ | `POST /api/estop` → `/emergency_stop` ← `Bool(true)` | `command_manager`가 직접 구독(arbiter 우회) → `hard_stop`, `motors_on=false`, `McuCommand.emergency_stop=true`.<br>**락 예외** — 제어권이 없어도 누를 수 있다 |
| **정지 해제** `#releaseEstop`<br>(문구 "정지 해제" 입력) | **B** ✅ | `POST /api/estop/release` → `/emergency_stop/release` (`alm_msgs/srv/ReleaseEstop`) | ✅ 래치 구현됨(`estop_latch: true`). **토픽에 `false` 를 흘려서는 안 풀린다.** MCU 가 fault/estop 을 보고 중이면 해제를 **거부**하고 사유를 응답 |
| **제어권** `#controlRoleButton` | **B** ✅ | `POST /api/session/acquire` / `release` + 5초 하트비트 | 웹 세션 제어권 ≠ 동작권(계획서 §2.1). 리스 TTL 15초 — 브라우저가 죽어도 락이 영구히 잠기지 않음 |
| **프로필** `#profilePill` | **L** | 설정 드로어 열기 | |
| **활성 맵** `#mapPill` | **L→B** | 모달 열 때 `GET /api/maps` | |
| **설정** `#openSettings` / `#closeSettings` | **L** | | |
| **설정 저장** `#saveSettings` | **B** | 세션 프로필·활성 맵 갱신 (`POST /api/session`) | 다음 launch 인자에 반영 |
| **연결 테스트** `#testConnection` | **B** | backend 헬스체크 + `/mcu/state` 수신 확인 + bridge ping | 지금은 850 ms 대기 후 고정 문구 |
| **가이드** `#openGuide` | **L** | `guide.js` 투어 | |
| **탭 4개** `.nav-item` | **L** | 구독 on/off 최적화 대상 | 매핑 중 잠금이 실제로 동작하도록 수정 필요 |
| **ALM 로고** `#coldStartReopen` | **L** | 0번 화면 복귀 | |
| HUD `bridge`/`backend`/`mcu` 점등 | **F** | WS 연결 상태 / `/mcu/state` 수신 여부 | |
| 배터리, CPU 스트립 | **F** | `/mcu/state.battery_voltage`, 리소스 통계 토픽 | |
| (신규 필요) 동작권 배지 | **F** | `/cmd_arbiter/owner` (latched) — `auto`/`teleop`/`teleop(held)` | 계획서 §12-5 |

---

## 2. 01 · SLAM 매핑 탭

### 2.1 워크플로 버튼

| 요소 (id) | 경로 | Jetson에서 실행되는 것 |
|---|---|---|
| **SLAM 시작** `#startMapping` | **B** ✅ | `POST /api/mapping/start {map, overwrite}`<br>① `fastlio_mid360.yaml` 의 `map_file_path` 를 대상 맵의 `cloud.pcd` 로 재작성<br>② `ros2 launch alm_navigation slam.launch.py` (프로세스 그룹)<br>⚠ 대상 맵에 이미 `cloud.pcd` 가 있으면 **거부** — `overwrite:true` 를 명시해야 진행 |
| **SLAM 종료** `#stopMapping` | **B** ✅ | `POST /api/mapping/stop` → 프로세스 **그룹**에 SIGINT(10s) → SIGTERM(5s) → SIGKILL |
| **3D PCD 저장** `#savePcd` | **B** ✅ | `POST /api/mapping/save` → `/map_save` (`std_srvs/Trigger`)<br>⚠ **상류 함정**: 누적 점군이 비어 있으면 FAST-LIO 가 `pcl::IOException` 을 안 잡고 **죽는다**(exit -6). 서비스는 영영 응답하지 않으므로 backend 가 슬롯 사망을 감지해 즉시 사유를 반환한다 |
| **2D 변환 모달 열기** `#openPcd2Pgm` | **L** | 모달만 염. 사전조건은 `state.mappingSaved` 가 아니라 **`cloud.pcd` 실제 존재**(`/alm/map_inventory` 기준) |
| **변환 실행** `#runPgmJob` | **B** ✅ | **subprocess**: `pcd2pgm.py --pcd maps/<맵>/cloud.pcd --out maps/<맵>/grid --resolution 0.05 --z-min -0.3 --z-max 1.5 --min-points 1`<br>진행률을 지어내지 않고 **실제 stdout 줄**을 폴링으로 흘린다 |
| **FPFH DB 생성** `#buildFpfhDb` | **B** ✅ | **subprocess**: `fpfh_map_builder --map maps/<맵>/cloud.pcd --output-prefix maps/<맵>/fpfh_map --voxel 0.5 --normal-radius 1.0 --feature-radius 2.5 --z-min -0.35 --z-max 1.0` |
| **맵 관리** `#openMapManager` | **F** ✅ | `/alm/map_inventory` 구독. `map_manager`가 `maps/`를 스캔해 자산 존재·크기·짝맞음(stale)까지 판정해 보낸다. **backend 불필요** |
| **새 맵** `#newMapButton` → `#confirmNewMap` | **B** ✅ | `POST /api/maps {name, label}` → `maps/<이름>/manifest.yaml` 원자적 생성. 이름은 `^[A-Za-z0-9][A-Za-z0-9_-]{1,31}$` + realpath 로 maps/ 이탈 재확인 |
| **활성 맵 전환** `#settingsMapSelect` | **B** ✅ | `PUT /api/maps/active {name}` → `active.yaml` 원자적 재작성. **이미 떠 있는 launch 에는 반영되지 않는다**(다음 기동부터) |
| **로그 지우기** `#clearLogs` / **레벨 필터** `#logLevel` | **L** | 클라이언트 버퍼(120줄) 조작 |

**⚠ launch 범위 주의.** `ros2 launch alm_bringup slam.launch.py`는 `robot.launch.py`(description + lidar + ekf + **base_control** + **mcu_interface**)까지 함께 띄웁니다. 상시 스택이 이미 떠 있는 운용에서 이걸 그대로 부르면 `cmd_arbiter`/`command_manager`/`mcu_bridge`가 **중복 기동**됩니다 — 시리얼 포트 이중 점유로 이어지므로 위험합니다.

→ **코드로 강제됨.** `alm_web_backend/processes.py` 의 `LAUNCH_ALLOWLIST` 에는 `("alm_navigation", "slam.launch.py")` 만 있고, `LAUNCH_DENYLIST` 가 `alm_bringup` 의 `slam/robot/navigation.launch.py` 를 사유와 함께 명시적으로 막습니다. 웹에서 임의의 launch 파일을 띄울 수 있으면 그건 원격 코드 실행이므로, 목록에 없는 것은 기동하지 않습니다.

### 2.2 UI 기본값 ↔ 스크립트 기본값 대조

| UI 입력 필드 | 값 | CLI 인자 | 스크립트 기본값 | 일치 |
|---|---|---|---|---|
| Resolution `#pgmResolution` | 0.05 | `--resolution` | 0.05 | ✅ |
| Min points `#pgmMinPoints` | 1 | `--min-points` | 1 | ✅ |
| Z min `#pgmZMin` | −0.3 | `--z-min` | −0.3 | ✅ |
| Z max `#pgmZMax` | 1.0 | `--z-max` | **1.5** | ⚠ 불일치 |
| SC 로그 `ring=20 sector=60 radius=10.0m` | — | `--num-ring`/`--num-sector`/`--max-radius` | 20 / 60 / 10.0 | ✅ |

`pcd2pgm.py`의 z 밴드는 라이다 마운트 높이에 따라 조정이 필요합니다(스크립트 docstring 참조). UI 도움말에 "실행 후 출력되는 z 분포를 보고 지면 위 0.2~1.5 m로 맞출 것"을 넣어야 합니다.

### 2.3 뷰포트 (읽기)

| 요소 | 경로 | 대상 |
|---|---|---|
| 3D 포인트클라우드 캔버스 | **F** | FAST-LIO 등록 점군 (**토픽명 미확정**, `/cloud_registered` 추정) |
| `points` 카운터 | **F** | 위 메시지의 `width × height` |
| 경과 시간, `#mappingStateLabel` | **B** | backend가 launch 기동 시각 기준 계산 |
| **뷰 리셋** `#reset3d` / **탑다운** `#topView3d` | **L** | WebGL 카메라 조작 (지금은 토스트만) |
| 레이어 토글 `.viewport-toolbar .tool` ×4<br>(누적 맵 / 현재 스캔 / 경로 / 로봇) | **L** | 렌더러 레이어 on/off. 구독 자체를 끄면 대역폭도 절약 |

---

## 3. 02 · 측위 · 자율주행 탭

### 3.1 측위

| 요소 (id) | 경로 | Jetson에서 실행되는 것 |
|---|---|---|
| **초기위치 자동 탐색** `#autoLocalization` | **B** | `ros2 launch alm_navigation localization.launch.py auto_init:=true`<br>(`teaser_fpfh_localizer` + `transform_publisher` + `fastlio_localization`. 맵·DB 경로는 인자를 비우면 `maps/active.yaml` 에서 자동 조립) |
| **수동 초기위치** `#manualInitialPose` + 맵 클릭 | **L→B** | `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{...}"` |
| **재측위** `#relocalize` | **B** | `teaser_fpfh_localizer` **재기동** (일회성 노드라 성공 후 종료되는 것이 정상) |
| 측위 상태 패널 (누적/매칭/ICP/수렴) | **F** | ⚠ `teaser_fpfh_localizer`가 구조화된 상태를 발행하지 않음(로그로만 냄). `/icp_result` 수신 = 수렴으로 간접 판정하거나 노드에 상태 토픽을 추가해야 함 |
| 측위 후보 레이어 `#candidateLayer` | **F** | `/sc_candidates` (`geometry_msgs/PoseArray`) |
| fitness / pose 표시 | **F** | `/icp_result` |

**⚠ `teaser_fpfh_localizer`와 `icp_node`는 성공하면 스스로 종료하는 일회성 노드입니다.** "재측위"는 재실행을 뜻하므로 backend가 프로세스 수명을 관리해야 하고, 측위 성공 후 노드 목록에서 사라지는 것을 UI가 "죽었다"로 오해하지 않도록 처리해야 합니다.

### 3.2 웨이포인트 · 주행

| 요소 (id) | 경로 | Jetson에서 실행되는 것 |
|---|---|---|
| **웨이포인트 추가** `#addWaypointMode` + 맵 클릭 | **L** | 클라이언트 좌표 누적. `pixelToMap()`을 맵 YAML의 `resolution`/`origin` 기준으로 일반화 필요 |
| **세트 저장** `#saveWaypointSet` | **B** | `POST /api/waypoints/<맵>` — 파일 저장 |
| **세트 불러오기** `#loadWaypointSet` | **B** | `GET /api/waypoints/<맵>` (지금은 고정 3개 하드코딩) |
| **주행 시작** `#startNavigation` | **B** | Nav2 액션 goal 전송:<br>`ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [...]}"`<br>사전 기동: `ros2 launch alm_navigation navigation.launch.py map:=<맵>.yaml` |
| **일시정지 / 재개** `#pauseNavigation` | **B** | Nav2에 pause가 없음 → **cancel goal** 후 남은 웨이포인트를 backend가 보관, 재개 시 새 goal 전송 |
| **중단** `#cancelNavigation` | **B** | cancel goal. 이후 Nav2가 0 twist를 내거나, 끊기면 `command_manager`의 `cmd_timeout_sec` 0.5 s로 자동 정지 |
| **주행 모드** `#navDriveModes` (Normal/Spin/Crab/Auto) | **B** | `ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}"` — **자율 소스 쪽 모드** |
| 〃 Crab 버튼 | — | `disabled`. `command_manager`의 `auto_crab_enabled: false`와 대응 (지금은 HTML 하드코딩) |
| 진행률 / 남은 거리 / ETA | **F** | `FollowWaypoints` 액션 피드백 (backend가 WS로 중계) |
| **알람 지우기** `#clearAlarms` | **L** | |

### 3.3 지도 뷰

| 요소 | 경로 | 대상 |
|---|---|---|
| 정적 맵 `staticMapLayer` | **F/B** | `pcd2pgm` 산출 PGM/YAML (backend가 이미지로 서빙) |
| 로봇 자세 `robotLayer` | **F** | TF `map→odom→base_link` + `/Odometry` |
| 글로벌 경로 `globalPathLayer` | **F** | `/plan` (`nav_msgs/Path`) |
| 로컬 경로 `localPathLayer` | **F** | 로컬 컨트롤러 경로 토픽 |
| 라이다 스캔 `scanLayer` | **F** | `/scan` (`pointcloud_to_scan` 출력) |
| 로컬/글로벌 코스트맵 | **F** | `/local_costmap/costmap`, `/global_costmap/costmap` |
| 줌·핏·회전·패닝 `#zoomIn` `#zoomOut` `#fitMap` `#resetRotation` `#mapPanToggle` | **L** | 뷰 변환만 |
| 좌표 표시 `#mapCoordinates` | **L** | 마우스 위치 → map 좌표 변환 |

---

## 4. 03 · 수동주행 탭

| 요소 (id) | 경로 | Jetson에서 실행되는 것 |
|---|---|---|
| **수동주행 시작** `#enterManual` → 모달 `#confirmManual` | **B** | `ros2 service call /cmd_arbiter/set_owner alm_msgs/srv/SetControlOwner "{owner: teleop}"`<br>응답 `active_owner`로 UI 확정 |
| **수동주행 종료** `#exitManual` | **B** | `... "{owner: auto}"` |
| **데드맨 패드** `.drive-button` ×5<br>(전진/후진/좌/우/정지) | **B** | 누르는 동안 20 Hz로<br>`ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: ...}, angular: {z: ...}}"`<br>떼면 0 twist 1회 후 발행 중단 |
| **주행 모드** `#manualModeSelector` (Normal/Spin/Crab/Auto) | **B** | `ros2 topic pub /drive_mode_teleop std_msgs/msg/String "{data: 'spin'}"` |
| **속도 배율** `#speedMultiplier` (25/50/75/100%) | **L** | 브라우저에서 twist에 곱함. 절대 한계는 `command_manager`가 별도로 클램프 |
| 지령/실측 막대, 바퀴 속도 4개, 조향각 | **F** | `/mcu/state` — `measured_velocity`, `wheel_speed[4]`, `steer_angle[2]` |
| 모터 상태 `#motorState` | **F** | `/mcu/state.motors_enabled` |
| (신규 필요) 실제 적용 모드 | **F** | `/drive_mode/effective` — auto가 내부적으로 고른 모드 |
| (신규 필요) `teleop(held)` 표시 | **F** | `/cmd_arbiter/owner` |

### 4.1 데드맨 명령이 실제로 지나가는 경로

```
브라우저 pointerdown
  → backend (검증: 세션 제어권 보유? owner==teleop?)
    → /cmd_vel_teleop (Twist, 20 Hz)
      → cmd_arbiter   : owner==teleop 이면 통과. 0.5 s 끊기면 teleop(held) → 0 twist 유지
        → /cmd_vel_mux (50 Hz)
          → command_manager : 모드 제약 → 속도/가속 클램프 → cmd_timeout 0.5 s
                              → MCU fault / odom stale 검사 → 4WIS 변환
            → /mcu/command (McuCommand: steer_deg, speed_rpm, mode_id)
              → mcu_bridge → UART /dev/ttyTHS1 → STM32
```

**브라우저는 twist만 보냅니다.** 4WIS 변환(`fourwis_encode.py`)과 속도 제한은 `command_manager` 한 곳에서만 하며, 이를 backend나 브라우저에서 중복 구현하면 안 됩니다.

### 4.2 UI 하드코딩 값 ↔ `base_control.yaml` 대조

`commandFor()`([app.js:728](assets/app.js#L728))의 상수가 `command_manager` 파라미터와 **우연히 일치**합니다.

| UI 상수 | 값 | `command_manager` 파라미터 | 값 |
|---|---|---|---|
| 전진 계수 | 0.45 | `max_linear_x` | 0.45 |
| 후진 계수 | −0.15 | `min_linear_x` | −0.15 |
| 회전 계수 | 0.8 | `max_angular_z` | 0.8 |
| `#cmdYBar` 정규화 분모 | 0.3 | `max_linear_y` | 0.30 |

값이 맞더라도 **서버 파라미터를 읽어 오도록 교체해야 합니다.** 로봇 쪽만 바꾸면 UI가 조용히 어긋납니다.

### 4.3 이 탭의 선행 조건

- `base_control.launch.py` + `mcu_interface.launch.py` 기동 (상시 스택)
- `/cmd_arbiter/owner == teleop`
- `/mcu/state.motors_enabled == true`, `fault == false`
- `/emergency_stop` 비활성
- ⚠ **`base_control.yaml`의 `##CONFIRM##` 4WIS 상수 확정 전에는 실차 금지** — twist가 그대로 잘못된 rpm/조향각이 됩니다
- ⚠ **`keyboard_teleop.py`와 동시 실행 금지** — 같은 `/cmd_vel_teleop`을 두 퍼블리셔가 씀 (계획서 §3.5)

---

## 5. 04 · 시스템 모니터링 탭

| 요소 (id) | 경로 | Jetson에서 실행되는 것 |
|---|---|---|
| CPU/GPU/RAM/온도/전력 | **F** | Jetson 리소스 통계 토픽 (**신규 노드 필요** — `jtop`/`tegrastats` 파싱) |
| 배터리 전압·전류 | **F** | `/mcu/state.battery_voltage` / `.battery_current` |
| 안전 인터록 표시 | **F** | `/mcu/state`의 `emergency_stop` / `command_timeout` / `fault` / `fault_code` / `fault_text` |
| ROS 2 그래프 목록 | **B** | `ros2 topic list` + `ros2 topic hz` 상당 (backend가 주기 계산) |
| **프로세스 새로고침** `#refreshProcesses` | **B** | `GET /api/processes` — systemd 관리 시 `systemctl status`, 아니면 프로세스 스캔 |
| 로그 창 | **F** | `/rosout` 구독 (⚠ **XSS 이스케이프 선행 필수** — `fault_text` 포함) |
| **진단 스냅샷** `#exportSnapshot` | **L** | 브라우저에서 JSON 파일 다운로드. 마지막 수신값을 그대로 씀 — 연동 후에도 수정 불필요 |
| 네트워크 (dBm, 업/다운로드) | **F** | 신규 노드 또는 backend 엔드포인트 |

---

## 6. 요약 — 경로별 집계

| 경로 | 개수 | 내용 |
|---|---|---|
| **L** (로컬, 연동 불필요) | 약 25개 | 뷰 조작(줌·회전·레이어), 로그 필터, 모달 열기, 속도 배율, 스냅샷, 가이드 |
| **B** (backend 명령) | **21개** | 아래 표 |
| **F** (구독 표시) | 약 20종 | 토픽/액션 피드백 |

### 6.1 backend가 구현해야 하는 명령 21개

| # | 트리거 | 실행 형태 |
|---|---|---|
| 1 | E-STOP | 토픽 pub `/emergency_stop` |
| 2 | E-STOP 해제 | 토픽 pub 또는 신규 서비스 |
| 3 | 제어권 확보/반납 | backend 세션 API (ROS 아님) |
| 4 | 설정 저장 | backend 세션 API |
| 5 | 연결 테스트 | 헬스체크 |
| 6 | SLAM 시작 | launch 기동 |
| 7 | SLAM 종료 | 프로세스 종료 |
| 8 | PCD 저장 | 서비스 `/map_save` (`std_srvs/Trigger`) |
| 9 | PGM 변환 | **subprocess** `pcd2pgm.py` |
| 10 | FPFH DB 생성 | **subprocess** `fpfh_map_builder` |
| ~~11~~ | ~~맵 목록 조회~~ | **구현 완료** — `/alm/map_inventory` 구독 (backend 불필요) |
| 12 | 새 맵 생성 | 파일시스템 |
| 13 | 자동 측위 | launch (`auto_init:=true`) |
| 14 | 수동 초기위치 | 토픽 pub `/initialpose` |
| 15 | 재측위 | 프로세스 재기동 |
| 16 | 웨이포인트 세트 저장/불러오기 | 파일시스템 |
| 17 | 주행 시작 | 액션 goal `FollowWaypoints` |
| 18 | 일시정지/재개 | 액션 cancel + 재전송 |
| 19 | 주행 중단 | 액션 cancel |
| 20 | 자율 주행 모드 변경 | 토픽 pub `/drive_mode` |
| 21 | 수동주행 진입/종료 + 데드맨 + 모드 | 서비스 `/cmd_arbiter/set_owner` + 토픽 pub `/cmd_vel_teleop`, `/drive_mode_teleop` |

**ROS 인터페이스 종류로는 6가지뿐입니다**: 서비스 2종(`/map_save`, `/cmd_arbiter/set_owner`), 토픽 pub 4종(`/emergency_stop`, `/initialpose`, `/cmd_vel_teleop`, `/drive_mode*`), 액션 1종(`FollowWaypoints`), subprocess 2개, launch 관리, 파일시스템 API. backend를 얇게 유지할 수 있는 이유입니다.

---

## 7. 선행 조건 체인

버튼이 눌릴 수 있으려면 무엇이 떠 있어야 하는가.

```
[상시]  robot.launch.py
          ├ description / lidar(livox, imu_relay, pointcloud_to_scan)
          ├ ekf (주행 모드에선 끔)
          ├ base_control (cmd_arbiter + command_manager)   ← 수동주행·자율주행 전제
          └ mcu_interface (mcu_bridge)                     ← 시리얼 유일 소유자
                │
    ┌───────────┼─────────────────────────┐
    ▼           ▼                         ▼
[매핑]      [측위]                    [자율주행]
slam.       localization.launch.py    navigation.launch.py
launch.py   (teaser_fpfh_localizer)   (map_server, Nav2)
    │           │                         │
    ▼           ▼                         ▼
 /map_save   /icp_result 수렴          FollowWaypoints
    │        = 초기위치 확정                 ▲
    ▼                                      │
 maps/<맵>/cloud.pcd                       │
    │                                      │
    ├─ pcd2pgm.py ─────→ grid.pgm/yaml ────┘ (map 인자)
    │
    └─ fpfh_map_builder ─→ fpfh_map*  ──→ fpfh_db_prefix

  세 자산의 짝맞음은 map_manager 가 감시해 /alm/map_inventory 로 알린다
  (fpfh_map.meta 의 map_fingerprint = 부모 cloud.pcd 의 FNV-1a).
```

UI가 이미 강제하는 순서(`savePcd` 없이는 변환·DB 불가, 측위 수렴 없이는 주행 불가)는 이 체인과 정확히 일치합니다. 연동 시 이 게이팅을 **서버 측에서도** 재검사해야 합니다.

---

## 8. 안전 경계

- **Foxglove Bridge 쓰기 차단** — 브라우저가 bridge로 직접 퍼블리시할 수 있으면 backend 검증과 arbiter 동작권을 통째로 우회합니다. 주의할 점은 **차단이 기본값이 아니라는 것**입니다: `client_topic_whitelist` 기본값이 `['.*']`이라 아무 토픽에나 발행이 열려 있습니다. 게다가 `topic_publish_whitelist` 같은 존재하지 않는 이름을 적으면 조용히 무시됩니다. 실제 적용은 `alm_bringup/config/foxglove_webui.yaml` 참조 — `capabilities`에서 `clientPublish`를 빼고, whitelist 4종을 `["$^"]`로 잠급니다. 변경 후에는 반드시 `ros2 param get`으로 반영을 확인하세요.
- **backend에서 재검사할 것**: 세션 제어권 보유 여부, `/cmd_arbiter/owner` 일치 여부, E-STOP 비활성, 시스템 상태(매핑 중 주행 명령 거부).
- **backend에서 하지 말 것**: 속도 클램프, 4WIS 변환, 타임아웃 정지. 전부 `command_manager`의 책임이며 두 곳에 두면 서로 다른 결론을 냅니다.
- **최종 안전 책임**은 브라우저 JS가 아니라 `cmd_arbiter`의 `teleop_timeout_sec`(0.5 s)과 `command_manager`의 `cmd_timeout_sec`(0.5 s), 그리고 MCU 펌웨어에 있습니다.
