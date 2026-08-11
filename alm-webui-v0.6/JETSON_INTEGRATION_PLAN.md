# ALM WebUI ↔ Jetson(ROS2) 실전 연동 계획

이 문서는 지금까지 프런트엔드 프로토타입으로만 존재하는 ALM WebUI를, 실제 Jetson Orin Nano + ROS2 로봇과 연동해 화면의 모든 동작이 진짜 데이터로 움직이게 만들기 위한 체크리스트와 로드맵입니다.

> **갱신 이력**
> - **2026-07-26 (초판)** — `index.html`/`assets/app.js`의 목업에 등장하는 노드·토픽 이름을 설계 의도로 간주하고 작성. 워크스페이스 실물과 대조하지 않음.
> - **2026-08-07 (맵 자산)** — 맵을 **폴더 단위**로 재편(`maps/<맵이름>/`)하고 `map_manager`(`/alm/map_inventory`)를 추가. UI의 하드코딩 맵 목록과 Scan Context·방식 A/B/C 잔재를 전부 제거. §0.1 신설.
> - **2026-08-06 (개정)** — `ALM_auto_ws/src` 실물과 전면 대조. 초판이 가정한 `command_gateway`가 실제로는 **`cmd_arbiter` + `command_manager` + `mcu_bridge` 3단 체인**으로 이미 구현돼 있음을 반영. §2 아키텍처, §3.5, §3.6, §4 인벤토리, §6 매핑표를 실측값으로 교체하고 §12(불일치 목록)를 신설.

---

## 0. 문서 목적

- 무엇을 실데이터로 바꿔야 하는지 빠짐없이 나열
- Jetson(로봇) 쪽과 브라우저(WebUI) 쪽 작업을 분리해서 각각 체크리스트로 관리
- 안전 critical 경로(E-STOP, 데드맨)는 마지막에 가장 신중하게 연동하도록 순서를 강제
- 실 데이터가 들어오면 즉시 위험해지는 기존 이슈(innerHTML 등)를 연동 전 처리 항목으로 명시
- **UI가 이름만 불러온 노드 중 무엇이 실재하고 무엇이 아직 없는지 구분** (개정 추가)

---

## 0.1 맵 자산 레이아웃 (2026-08-07 신설)

**폴더 하나 = 맵 하나.** `manifest.yaml` 이 있는 디렉터리만 맵으로 인정한다
(실험 잔재 폴더가 목록에 섞이는 것을 막기 위해서다).

```
ALM_auto_ws/src/alm_navigation/maps/
├── active.yaml                 active: alm_lab   ← launch 와 map_manager 가 읽는다
├── alm_lab/
│   ├── manifest.yaml           이름·라벨·생성일 (자산 '사실'은 적지 않는다)
│   ├── cloud.pcd               3D 점군   — FAST-LIO /map_save
│   ├── grid.pgm  grid.yaml     2D 격자   — pcd2pgm.py
│   └── fpfh_map.meta  fpfh_map_{points,normals,fpfh}.pcd   — fpfh_map_builder
└── empty_test/                 자산 없는 맵 (placeholder 화면 확인용, 지워도 됨)
```

### 왜 이렇게 했나

평평한 구조에서는 맵이 둘 이상이 되는 순간 **"이 pgm 이 어느 pcd 의 자식인가"**
를 알 수 없었다. 새 구조에서는 폴더가 곧 그 답이다.

### 짝 맞음(stale) 판정 — 이미 파일에 근거가 있었다

`fpfh_map.meta` 에는 부모 PCD 의 지문이 들어 있고(`map_fingerprint`),
`teaser_fpfh_localizer` 는 기동할 때 이것을 실제로 검증한다
(`verify_map_fingerprint`, `src/teaser_fpfh_localizer.cpp:225`).
`map_manager` 는 같은 근거로 **미리** 판정해 화면에 띄운다.

| 검사 | 근거 | 비용 |
|---|---|---|
| 점 개수 대조 | `meta.map_input_points` vs `cloud.pcd` 헤더 `POINTS` | 헤더 4 KB만 읽음 |
| mtime 역전 | `cloud.pcd` 가 `grid.pgm`/`fpfh_map.meta` 보다 최신인가 | stat 3회 |
| 지문 대조 | FNV-1a 64 | 52 MB ≈ 14초 → **기본 off**, 켜면 백그라운드 + 캐시 |

> ⚠ **지문 상수 함정.** `icp_relocalization` 의 FNV-1a 오프셋은
> `1469598103934665603` 으로, 교과서 값(`14695981039346656037`)에서 한 자리가
> 빠진 오타다. builder 와 localizer 가 같은 함수를 쓰므로 체크섬으로는 일관되게
> 동작하지만, 다른 구현에서 교과서 값을 쓰면 **멀쩡한 DB 를 전부 불일치로
> 오판한다.** `map_manager.py` 는 이 값을 그대로 맞춰 두었다.

### 경로 조립

`alm_navigation/launch/map_layout.py`(공용 헬퍼)가 `active.yaml` 을 읽어
`cloud` / `grid_yaml` / `fpfh_prefix` 경로를 만든다. launch 들은 인자를 비우면
활성 맵을 자동으로 쓰고, 인자로 덮어쓸 수도 있다.

> ⚠ **`--symlink-install` 함정.** colcon 은 `maps/` 를 실제 디렉터리로 만들고
> 그 안의 항목만 심링크한다. 그래서 share 경로를 그대로 쓰면 매핑으로 소스 쪽에
> 새 맵 폴더가 생겨도 `colcon build` 전까지 보이지 않는다. `maps_root()` 가
> 심링크를 따라 원본 디렉터리로 되돌리는 이유다.

> ⚠ **절대경로 2곳.** `fastlio_mid360.yaml` 의 `map_file_path` 와
> `fastlio_relocalization.yaml` 의 `prior_map_path` 는 fast_lio 자신의
> 파라미터라 launch 치환이 닿지 않는다. **새 맵으로 매핑하기 전에 직접 바꿔야**
> 기존 맵을 덮어쓰지 않는다.

---

## 1. 지금 상태 요약 — 무엇이 진짜고 무엇이 가짜인가

### 1.1 브라우저 쪽 (전부 mock)

| 화면 요소 | 지금 상태 (mock) | 연동 시 필요한 것 |
|---|---|---|
| 3D 포인트클라우드 뷰포트 | `drawPointCloud()`가 sin/cos 공식으로 매 프레임 점을 생성하는 Canvas 2D 눈속임 | 실제 `PointCloud2` 메시지 구독 + WebGL 3D 렌더러로 전면 교체 |
| 2D 운용 지도 | SVG에 좌표가 하드코딩된 고정 도형 | 실제 PGM/YAML 지도 이미지 + OccupancyGrid 코스트맵 |
| SLAM 시작/종료, 저장, 변환, DB 생성 | `setTimeout` 기반 진행률 시뮬레이션 | launch 기동 RPC + `/map_save` 서비스 + **CLI 스크립트 실행 래핑**(§3.3 참조) |
| 초기위치/측위 | 정해진 시나리오대로 상태가 자동 전환 | `teaser_fpfh_localizer` 노드의 실제 진행 상황 구독 |
| 웨이포인트 · 자율주행 | 클라이언트에서 진행률을 랜덤 증가시킴 | Nav2 `FollowWaypoints` 액션 연동 |
| 수동주행(데드맨) | `commandFor()`가 계산한 값을 화면에만 표시 | `/cmd_vel_teleop` 발행 + `/mcu/state` 피드백 구독 |
| 모니터링(CPU/GPU/온도/네트워크 등) | `updateMetrics()`가 매초 랜덤값 생성 | Jetson 실측 통계를 토픽으로 발행받아 구독 |
| 안전 인터록 · E-STOP | 클라이언트 `state.estop` 불리언 하나로 전부 표현 | `/emergency_stop` 발행 + `/mcu/state` 구독 (§3.6) |
| 맵 관리(`state.maps`) | 클라이언트 배열, 새로고침하면 초기화 | 백엔드 API로 실제 파일 시스템 조회 |

브라우저에는 **네트워크 코드가 한 줄도 없습니다.** `fetch`/`WebSocket` 호출이 전무하고, 유일한 외부 통신은 CDN 웹폰트 2개(`index.html:10,14`)입니다.

### 1.2 로봇 쪽 — UI가 부르는 이름 vs `ALM_auto_ws` 실물 (개정 신설)

| UI/초판 문서가 부르는 이름 | 실재 여부 | 실제 대응물 |
|---|---|---|
| `command_gateway` | ❌ 그 이름으론 없음 | **`cmd_arbiter` + `command_manager` + `mcu_bridge`** 3단 체인이 이미 담당 |
| `safety_supervisor` | ❌ 없음 | 안전 게이팅은 `command_manager` 안에 들어가 있음. 상태 집계 발행자만 없음 |
| `alm_web_backend` | ❌ 없음 | 신규 작성 필요 (§3.7) |
| `foxglove_bridge` | ❌ 미설치 | 신규 도입 필요 (§3.8) |
| `jetson_stats_publisher` | ❌ 없음 | 신규 작성 필요 (§3.9) |
| `teaser_fpfh_localizer` | ✅ 존재 | `alm_navigation/scripts/teaser_fpfh_localizer` |
| `livox_udp_pointcloud2` | ✅ 존재 | `alm_sensors/scripts/livox_udp_pointcloud2.py` |
| `/map_save` | ✅ 존재 | FAST-LIO 서비스, 경로는 `fastlio_mid360.yaml`의 `map_file_path` 고정 |
| `pcd2pgm` / `sc_build_db` | ⚠ 스크립트만 | **ROS 서비스가 아니라 argparse CLI 오프라인 스크립트** |

**핵심 수정**: 초판이 "신규 작성 필요"로 잡았던 명령 경로는 이미 존재하며, 게다가 UI보다 안전 모델이 더 정교합니다. 반대로 초판이 "이름이 이미 확정됨"으로 낙관한 `safety_supervisor`/`command_gateway`는 그 이름으로 만들면 안 됩니다 — 실물 이름에 UI를 맞추는 편이 맞습니다.

---

## 2. 목표 아키텍처 (개정)

```
┌───────────────────────────── Jetson Orin Nano ──────────────────────────────┐
│                                                                              │
│  [센서]  livox_udp_pointcloud2 ─▶ /livox/lidar (5 Hz)                        │
│          livox_udp_imu ─▶ /livox/imu (200 Hz) ─▶ imu_relay ─▶ /imu/data      │
│          pointcloud_to_scan ─▶ /scan                                         │
│                                                                              │
│  [매핑]  fast_lio/fastlio_mapping ─▶ /Odometry, 등록점군, 서비스 /map_save   │
│  [측위]  teaser_fpfh_localizer ─▶ /initialpose, /sc_candidates                        │
│          icp_relocalization(transform_publisher) ─▶ TF map→odom              │
│  [주행]  Nav2 ─▶ costmap, /plan, FollowWaypoints 액션, /cmd_vel              │
│                                                                              │
│  ── 명령 체인 (이미 구현됨. 시리얼은 mcu_bridge 하나만 소유) ──               │
│                                                                              │
│    Nav2 ─/cmd_vel──────────┐                                                │
│          ─/drive_mode──────┤                                                │
│   teleop ─/cmd_vel_teleop──┼─▶ [cmd_arbiter] ─/cmd_vel_mux────┐             │
│          ─/drive_mode_teleop┘   50 Hz, owner=auto|teleop      │             │
│                                 └─▶ /cmd_arbiter/owner(latched)│             │
│                                 └◀── 서비스 /cmd_arbiter/set_owner           │
│                                                                │             │
│                                                                ▼             │
│                          /emergency_stop(Bool) ─▶ [command_manager] 50 Hz    │
│                          (arbiter 우회, 직접 구독)   4WIS 변환 + 안전 게이팅  │
│                                                       ─▶ /mcu/command        │
│                                                       ─▶ /drive_mode/effective│
│                                                                ▼             │
│                                              [mcu_bridge] /dev/ttyTHS1       │
│                                              ─▶ /mcu/state, /wheel_odom      │
│                                                                ▼ UART        │
│                                                             STM32 4WIS       │
│                                                                              │
│  jetson_stats_publisher(신규) ──▶ CPU/GPU/온도/전력 토픽                      │
│                                                                              │
│         │ (모든 토픽/서비스/액션)                                            │
│         ▼                                                                    │
│  ┌───────────────────┐        ┌──────────────────────────────┐              │
│  │  foxglove_bridge   │        │   alm_web_backend (신규)      │              │
│  │  (읽기 전용 스트림) │        │  얇은 어댑터:                 │              │
│  │       [신규 도입]   │        │   · 웹 세션 제어권 관리        │              │
│  │                    │        │   · set_owner / 액션 / 서비스  │              │
│  │                    │        │   · launch 기동·종료, 파일 조회│              │
│  └─────────┬─────────┘        └──────────┬───────────────────┘              │
└────────────┼──────────────────────────────┼─────────────────────────────────┘
             │ wss:// (텔레메트리)            │ https:// (명령)
             ▼                              ▼
        ┌────────────────────────────────────────┐
        │              브라우저 (WebUI)            │
        │  ros-bridge.js  ──  renderer3d.js        │
        │  (구독 전용)         (WebGL 렌더링)        │
        │  app.js state는 "서버 값의 캐시"로 전환    │
        └────────────────────────────────────────┘
```

**핵심 결정**: 브라우저는 ROS2 노드가 될 수 없고, ROS2 액션(FollowWaypoints 등)도 웹소켓 프로토콜만으로 다루기 번거롭습니다. 그래서 역할을 둘로 쪼갭니다.

- **foxglove_bridge**: 고빈도 텔레메트리(포인트클라우드, Odometry, 코스트맵, `/mcu/state`)를 브라우저로 스트리밍 — 읽기 전용
- **alm_web_backend**: 브라우저의 단순한 명령(HTTP/WS)을 ROS2 서비스·액션 호출로 "번역"하는 중간 계층 — 쓰기(명령) 전담

**개정 사항 — backend의 몸집이 줄었습니다.** 초판은 backend가 검증·레이트리밋·타임아웃까지 책임지는 두꺼운 계층을 상정했지만, 그 역할은 이미 `cmd_arbiter`(동작권·freshness)와 `command_manager`(속도/가속 제한, cmd timeout, MCU fault, odom 워치독)가 수행합니다. backend는 **웹 세션 제어권 관리 + 프로토콜 번역**만 하는 얇은 어댑터로 설계하고, **안전 판단을 backend에 중복 구현하지 않습니다.**

### 2.1 "제어권"이 두 개라는 점을 혼동하지 말 것 (개정 신설)

UI의 HUD 제어권 배지와 `cmd_arbiter`의 동작권은 **서로 다른 축**입니다. 이걸 하나로 합치면 다중 접속에서 반드시 사고가 납니다.

| 축 | 질문 | 관리 주체 | 값 |
|---|---|---|---|
| **웹 세션 제어권** | 접속한 여러 브라우저 중 *누가* 조작할 수 있나 | `alm_web_backend` (신규) | 세션 ID 하나 or 없음(전원 관전) |
| **동작권(ownership)** | 로봇이 *자율*을 따르나 *텔레옵*을 따르나 | `cmd_arbiter` (구현됨) | `auto` / `teleop` / `teleop(held)` |

- UI `toggleControl()`([app.js:209](assets/app.js#L209)) → **웹 세션 제어권**. backend가 판정, 클라이언트 `state.hasControl`은 캐시.
- UI `enterManual()`([app.js:701](assets/app.js#L701)) → **동작권**. `set_owner("teleop")` 호출에 대응.
- UI `exitManual()` → `set_owner("auto")`.

---

## 3. Jetson(로봇) 측 체크리스트

### 3.1 OS/네트워크 기본
- [ ] JetPack 버전, ROS2 배포판(Humble/Iron 등) 확정 및 설치
- [ ] 고정 IP 또는 mDNS 호스트명 할당 — 설정 드로어의 `https://192.168.1.110`, `wss://192.168.1.110/foxglove` ([index.html:733-734](index.html#L733-L734))를 실제 값으로 교체
- [ ] `ROS_DOMAIN_ID` 통일 (센서/매핑/Nav2/백엔드 프로세스 전부 동일 도메인)
- [ ] 방화벽에서 foxglove_bridge 포트(기본 8765)와 backend 포트 개방
- [ ] NTP/chrony로 시간 동기화 — 로그·토픽 타임스탬프 정합성 확보

### 3.2 센서 드라이버 — 대부분 확인 완료
- [x] `alm_sensors/launch/lidar.launch.py` 존재. 기동 노드: `livox_lidar`(`livox_udp_pointcloud2.py`), `livox_imu`(`livox_udp_imu.py`), `imu_relay`, `pointcloud_to_scan`, `base_to_livox_tf`(static TF)
- [x] 토픽 확정: `/livox/lidar`(5 Hz), `/livox/imu`(200 Hz), `/imu/data`(relay 출력), `/scan`(2D 투영)
- [ ] 실기에서 실제 발행 주기 측정 (`ros2 topic hz`) — UI HUD의 "5.0 Hz" 표기와 일치시킬 것
- [ ] 패킷 손실률, pts/s를 계산해 토픽으로 노출하는 로직 (지금은 랜덤값). `livox_udp_pointcloud2.py`가 UDP를 직접 파싱하므로 여기에 카운터를 붙이는 게 가장 싸다
- [ ] MID-360 상태 패킷 파서 — UI가 이미 "status_available=false"로 미지원 표기 중([app.js:1025](assets/app.js#L1025)). 지원할지 영구 미지원으로 확정할지 결정

### 3.3 매핑/측위 스택 — 중대한 정정 있음

- [x] `fast_lio` 확인 완료 — `install/fast_lio/lib/fast_lio/fastlio_mapping` 로 **빌드돼 있음**. 소스는 `src/thirdparty`가 아니라 **워크스페이스 루트 `thirdparty/`** 에 있고 그쪽은 `COLCON_IGNORE` 상태(별도 빌드). `icp_relocalization`, `livox_ros_driver2`, `teaserpp`, `lio_sam` 도 동일하게 install 에 존재
  - [ ] 루트 `thirdparty/`의 빌드 절차를 README 나 SETUP_CHECKLIST 에 명시할 것 — 클린 체크아웃에서 `colcon build` 만으로는 재현되지 않음
- [ ] **FAST-LIO의 등록 점군 출력 토픽명 확정** — 3D 뷰포트가 구독할 대상. 통상 `/cloud_registered`(map 프레임) / `/cloud_registered_body`이지만 포크마다 다름. `ros2 topic list` 실측으로 확정
- [x] `/Odometry` — `command_manager`의 odom 워치독이 이미 이 토픽을 구독 중(`base_control.yaml`). 존재 확정
- [x] `/map_save` 서비스 — FAST-LIO 제공. **타입은 `std_srvs/srv/Trigger`로 확정**(`alm_bringup/launch/slam.launch.py` docstring). 저장 경로는 `fastlio_mid360.yaml`의 `map_file_path`에 **고정**되어 있음
  - [ ] ⚠ UI는 맵 이름별 경로(`/data/maps/{맵이름}/...`)를 전제. 실물은 단일 고정 경로 → **backend가 저장 후 맵 이름 디렉터리로 옮기거나, 호출 전 파라미터를 바꿔야 함**. 어느 쪽으로 할지 결정 필요
  - [ ] `pcd_save_en: true`가 아니면 빈 pcd가 나온다는 점(설정 파일 주석)을 backend가 사전 검사
- [ ] **`pcd2pgm.py`와 `sc_build_db.py`는 ROS 서비스가 아니라 argparse CLI 오프라인 스크립트입니다.** 초판의 "서비스 신규 정의" 항목은 다음 둘 중 하나로 대체:
  - **(권장) backend가 subprocess로 실행**하고 stdout을 진행률로 파싱 — 스크립트를 건드리지 않아 CLI 운용과 웹 운용이 갈라지지 않음
  - 스크립트를 ROS 서비스 노드로 감싸기 — 인터페이스는 깔끔하나 코드가 이중화됨
- [ ] `pcd2pgm.py` 파라미터(`--z-min`/`--z-max`/`--resolution`/`--min-points`)를 UI 변환 모달의 입력 필드와 이름·단위까지 맞출 것. z 밴드는 라이다 마운트 높이에 따라 조정이 필요하다는 점을 UI 도움말에 넣어야 함(스크립트 docstring 참조)
- [x] `teaser_fpfh_localizer` 확인 완료 — 파라미터 `map_pcd`, `fpfh_db_prefix`, `accum_frames`(10), `verify_map_fingerprint`(true). 발행 `/icp_result`. 구독 `/livox/lidar`.
      ※ `sc_localizer.py` / `sc_build_db.py` / `scan_context.py` 는 `bbc6dad` 에서 **삭제**됐다 — 이 문서의 예전 SC 서술은 더 이상 유효하지 않다.
  - [ ] ⚠ **상태 머신(COLLECT→MATCH→WAIT)을 토픽으로 노출하지 않습니다.** UI 측위 패널의 단계별 진행 표시를 쓰려면 상태 발행을 추가하거나(권장, 소규모 수정), UI를 "탐색 중/성공/실패" 3단계로 단순화해야 함
  - [ ] ⚠ **성공 시 노드가 스스로 종료합니다(일회성).** `icp_node`도 마찬가지. UI의 "재측위" 버튼은 노드 재기동을 의미하므로 backend가 재실행을 담당해야 함
- [x] 측위 방식은 **FPFH+TEASER++ 하나로 확정**(bbc6dad). `localization.launch.py`가 `transform_publisher` + `teaser_fpfh_localizer` + `fastlio_localization`을 기동하며, 맵·DB 경로는 인자를 비우면 `maps/active.yaml`에서 조립된다. UI의 방식 A/B/C 선택은 제거했다.

### 3.4 자율주행(Nav2) 스택
- [ ] Nav2 설치 및 파라미터 구성 (`alm_navigation/config/nav2.yaml` 존재, 실기 검증 필요)
- [ ] `/initialpose`(`geometry_msgs/PoseWithCovarianceStamped`)로 수동 초기위치 지정 가능한지 확인 — `teaser_fpfh_localizer`가 이미 이 토픽을 쓰므로 규약 일치
- [ ] `FollowWaypoints` 액션 서버 정상 동작 및 웨이포인트 배열 전달 형식 확인
- [ ] `/global_costmap/costmap`, `/local_costmap/costmap` (`nav_msgs/OccupancyGrid`) 실제 발행 확인
- [ ] 글로벌/로컬 경로 토픽(`/plan`, 로컬 컨트롤러의 경로 토픽) 확인
- [ ] **Nav2 출력이 `/cmd_vel`이고 이게 `cmd_arbiter`의 nav 입력입니다.** Nav2를 직접 `command_manager`에 물리면 안 됨 — 반드시 arbiter 경유
- [ ] **웨이포인트 반복 횟수 · 순환 주행은 Nav2 기본 기능이 아님** — `alm_web_backend`가 `FollowWaypoints`를 반복 호출하는 방식으로 직접 구현해야 함

### 3.5 수동주행/제어 경로 — 전면 개정 (이미 구현됨)

초판의 `command_gateway` 단일 노드 가정을 폐기하고, 실물 3단 체인 기준으로 다시 씁니다. 상세 설계는 [`docs/control_arbitration.md`](../docs/control_arbitration.md) 참조.

**`cmd_arbiter`** (`alm_base_control/scripts/cmd_arbiter.py`)
- [x] 자율 소스(`/cmd_vel`, `/drive_mode`)와 텔레옵 소스(`/cmd_vel_teleop`, `/drive_mode_teleop`) 중 동작권 보유 쪽만 `/cmd_vel_mux`, `/drive_mode_mux`로 통과. 50 Hz 재발행
- [x] 서비스 `/cmd_arbiter/set_owner` (`alm_msgs/srv/SetControlOwner`, 요청 `owner`, 응답 `success`/`message`/`active_owner`)
- [x] latched 토픽 `/cmd_arbiter/owner` — 값: `auto` / `teleop` / `teleop(held)`
- [x] **인계 안전**: `teleop` 중 명령이 `teleop_timeout_sec`(0.5s) 끊기면 `teleop(held)`로 들어가 0 twist를 유지하고 **자율로 자동 복귀하지 않음**. `set_owner("auto")` 명시 호출로만 복귀
- [ ] ⚠ **`teleop(held)` 상태를 UI가 표현할 자리가 없습니다.** HUD 또는 수동주행 탭에 "정지 유지 — 자율 복귀하려면 명시 해제" 상태를 추가할 것. 이걸 빼면 조작자가 "왜 자율이 다시 안 도나"를 알 수 없음
- [ ] ⚠ **웹 텔레옵과 `keyboard_teleop.py`가 같은 `/cmd_vel_teleop`에 물리면 두 퍼블리셔가 경쟁합니다.** 둘 중 하나로 결정:
  - (a) backend를 `/cmd_vel_teleop`의 유일한 웹 퍼블리셔로 두고, `keyboard_teleop`은 현장 디버그 전용 — 동시 사용 금지를 문서·기동 스크립트로 강제
  - (b) `cmd_arbiter`에 `web` 소스를 3번째 owner로 추가 — 코드 수정 필요하지만 경쟁이 구조적으로 사라짐. **웹 운용을 상시로 할 거라면 이쪽 권장**

**`command_manager`** (`alm_base_control/scripts/command_manager.py`)
- [x] `/cmd_vel_mux` + `/drive_mode_mux` + `/emergency_stop`(Bool) + `/mcu/state` + `/Odometry` 구독 → `/mcu/command`(`alm_msgs/McuCommand`) 50 Hz 발행. 부수 발행 `/drive_mode/effective`(String)
- [x] 안전 게이팅 전부 여기 있음: `cmd_timeout_sec` 0.5 / `odom_watchdog_sec` 0.5 / `stop_on_mcu_fault` / 속도·가속 제한
- [x] 4WIS 변환(twist → `steer_deg`/`speed_rpm`/`mode_id`)도 여기 한 곳에서만. 텔레옵은 twist만 보냄 — **웹도 twist만 보내면 됩니다**
- [ ] ⚠ **`base_control.yaml`의 `##CONFIRM##` 항목이 미확정**입니다: `wheelbase_m`, `track_m`, `rws_ratio`, `wheel_radius_m`, `gear_ratio`, `max_steer_deg`, `straight_angle_deg`, `crab_rpm_scale`, `zero_turn_rpm_scale`, `crab_steer_sign`, `spin_steer_sign`, `max_rpm`. **이 값이 틀리면 웹에서 보낸 twist가 엉뚱한 rpm/조향각이 됩니다.** 웹 수동주행(Phase 5) 착수 전 실차 확정 필수
- [ ] 속도 한계값을 UI에 하드코딩하지 말고 backend가 파라미터를 읽어 내려줄 것 (§12 참조)

**`mcu_bridge`** (`alm_mcu_interface/scripts/mcu_bridge.py`)
- [x] `/dev/ttyTHS1` @115200의 **유일한 소유자**. `/mcu/command` 구독 → UART. `/mcu/state`, `/wheel_odom`, `/joint_states` 발행
- [ ] `baudrate`가 STM32 펌웨어와 일치하는지 확인 (`##CONFIRM##` 표시됨)
- [ ] **웹 연동이 시리얼 포트를 절대 직접 열지 않음**을 backend 코드 주석에 명시. 두 프로세스가 같은 포트에 쓰면 프레임 동기가 깨짐

**`/mcu/state` 필드 — UI 수동주행 탭과 거의 1:1 대응 (확정됨)**

| `McuState` 필드 | UI 대응 |
|---|---|
| `measured_velocity` (Twist) | `#measuredX`, `#measuredZ` |
| `wheel_speed[4]` (FL/FR/RL/RR) | `#speedFL`~`#speedRR` |
| `steer_angle[2]` (**front, rear 2개**) | `#frontSteer`, `#rearSteer` |
| `battery_voltage`, `battery_current` | 배터리 위젯, 모니터링 탭 |
| `motors_enabled` | `#motorState` |
| `emergency_stop`, `command_timeout`, `fault`, `fault_code`, `fault_text` | 안전 스트립, 알람 목록 |

- [ ] ⚠ UI는 바퀴 4개의 조향각을 각각 회전시키지만(`#wheelFL`~`#wheelRR`, [app.js:777](assets/app.js#L777)), **실제 조향은 축당 2개(타이로드 연동)라 값이 2개뿐**입니다. UI를 축 단위 표현으로 고칠 것

### 3.6 안전 계통 — 개정

- [x] **E-STOP 경로는 이미 존재**: `/emergency_stop`(`std_msgs/Bool`)을 `command_manager`가 **직접 구독**(arbiter 우회)하므로 동작권이 누구에게 있든 즉시 정지
- [ ] ⚠ **래치 의미 불일치.** `command_manager._on_estop()`은 `self.estop = bool(msg.data)`로 토픽 값을 그대로 따라갑니다 — **래치가 아닙니다.** 반면 UI는 "확인 문구를 입력해야 해제되는 래치"를 전제([app.js:190](assets/app.js#L190)). 셋 중 하나로 결정:
  - (a) `command_manager`에 래치 추가 + 해제 서비스 신설 — **가장 안전, 권장**
  - (b) backend가 래치를 들고 `False`를 안 보내는 방식 — backend가 죽으면 래치도 사라짐(위험)
  - (c) UI를 비래치 토글로 낮춤 — 권장하지 않음
- [ ] `safety_supervisor`는 **만들지 말고**, `/alm/safety_status` 집계 발행만 필요하면 별도 소형 노드나 backend가 `/mcu/state` + `/drive_mode/effective` + `/cmd_arbiter/owner`를 합쳐 만들 것. 안전 판단 로직을 두 곳에 두면 서로 다른 결론을 낼 수 있음
- [ ] **물리 E-STOP과 소프트웨어 E-STOP의 관계 확정** — 웹 UI 장애가 로봇을 못 멈추게 해서는 안 되고, 반대로 웹 UI만으로 물리 스위치를 대체해서도 안 됨
- [ ] UI가 표시하는 "감독 하트비트 5 Hz"의 실체 확정 — 현재 워크스페이스에 브라우저 하트비트 개념이 없음. `command_manager`의 `cmd_timeout_sec` 0.5초가 사실상의 하트비트 감시이므로, **UI 문구를 실물에 맞추거나** backend가 별도 하트비트를 구현할 것

### 3.7 웹 연동 백엔드 (alm_web_backend) — ✅ 구현됨 (2026-08-10)

**실물**: `ALM_auto_ws/src/alm_web_backend/` · 기본 포트 `8081` · `webui_dev.launch.py` 가 함께 띄운다.

```
alm_web_backend/
  http_server.py   ThreadingHTTPServer + 라우팅 + Bearer 인증 + CORS + SSE
  session.py       웹 세션 락 (리스 15초, 하트비트로 갱신)
  processes.py     launch 슬롯 (allowlist + 프로세스그룹 단계 종료)
  jobs.py          subprocess 작업 + stdout 링버퍼(500줄)
  maps_write.py    maps/ 쓰기 (manifest, active.yaml, 매핑 타깃)
  ros_iface.py     rclpy 노드 — 퍼블리셔 · 서비스 클라이언트
```

설계 결정 셋:

- **표준 라이브러리만.** 젯슨에 fastapi/uvicorn/aiohttp/flask 가 하나도 없다. 로봇 온보드에 pip 의존성을 새로 심으면 재현성이 나빠지고, 이건 엔드포인트 열댓 개짜리 어댑터지 웹앱이 아니다. uvicorn 의 asyncio 루프와 rclpy 실행기를 한 프로세스에 엮는 것보다 **실행기(메인) + HTTP 스레드풀** 이 단순하다.
- **fail-closed 인증.** `ALM_WEB_TOKEN` 이 없으면 기동을 **거부**한다. `--allow-no-auth` 를 명시해야만 예외.
- **HTTP 스레드에서 `spin_until_future_complete` 금지.** 같은 노드를 두 곳에서 spin 하면 콜백이 유실된다. `call_async` + `future.done()` 폴링을 쓴다 (`ros_iface._call`).

- [x] 역할 확정: rclpy 기반 ROS2 노드 + REST 서버. **얇은 어댑터**로 한정
  - 웹 세션 제어권 관리 (§2.1의 축 하나)
  - `set_owner` 서비스 / `FollowWaypoints` 액션 / `/map_save` 서비스 중계
  - `pcd2pgm.py`·`sc_build_db.py` subprocess 실행 및 진행률 스트리밍
  - launch 그룹 기동·종료, 맵 디렉터리 조회
  - `command_manager` 파라미터(속도 한계 등)를 읽어 UI에 내려주기
- [ ] **안전 판단(속도 제한, 타임아웃, fault 정지)을 여기에 중복 구현하지 않는다**를 원칙으로 명시
- [ ] 노출할 API 목록 설계 (§6 매핑표 기준으로 구체화)
- [ ] 프로세스 재시작 시 상태 복구 전략 — 재시작 시점에 매핑 중이었다면? `/cmd_arbiter/owner`가 latched라 동작권은 재조회로 복구 가능
- [ ] 실행 로그 파일 → 웹 UI 로그 창 스트리밍 방법 확정 (`/rosout` 구독이 가장 간단)

### 3.8 Foxglove Bridge
- [ ] `foxglove_bridge` 설치, 포트(기본 8765) 확인, `wss://` 사용 시 TLS 인증서 준비
- [x] `foxglove_bridge` **3.4.2 설치 완료**. 설정은 [`alm_bringup/config/foxglove_webui.yaml`](../ALM_auto_ws/src/alm_bringup/config/foxglove_webui.yaml)
- [x] 구독 allowlist 구성 (`topic_whitelist`) — 등록점군, `/Odometry`, `/mcu/state`, `/cmd_arbiter/owner`, `/drive_mode/effective`, costmap, `/plan`, `/sc_candidates`, TF 등 19개
- [x] **쓰기 차단** — 아래 함정 주의
- ⚠ **`topic_publish_whitelist` 라는 파라미터는 없습니다.** 클라이언트 발행을 제어하는 실제 이름은 **`client_topic_whitelist`** 이고, **기본값이 `['.*']` — 즉 브라우저가 아무 토픽에나 publish 할 수 있는 상태가 기본값**입니다. 없는 이름을 적으면 조용히 무시되므로 "막았다"고 착각하기 쉽습니다. 반드시 `ros2 param get /foxglove_bridge client_topic_whitelist` 로 실제 반영을 확인할 것
  - 현재 적용: `capabilities: ["connectionGraph"]` 로 `clientPublish` 자체를 제거(구조적 차단) + `client_topic_whitelist`/`service_whitelist`/`param_whitelist`/`asset_uri_allowlist` 를 전부 `["$^"]`(매칭 불가 정규식) 로 이중 차단
- [ ] 인증(토큰/비밀번호) 필요 여부 결정 — 공유 네트워크라면 필수로 간주

### 3.9 시스템 리소스 모니터링
- [ ] CPU/GPU/RAM/전력/온도 — `jtop`(jetson-stats) 또는 `tegrastats` 파싱 결과를 토픽으로 재발행하는 소형 노드 신규 작성
- [ ] 무선 신호 세기(dBm), 업/다운로드 속도 — 호스트 OS 네트워크 통계 조회 노드 또는 backend 엔드포인트
- [ ] 실행 프로세스 목록/PID — systemd로 관리한다면 backend가 `systemctl status` 조회, 아니면 프로세스 감시 스크립트

---

## 4. 토픽 · 서비스 · 액션 인벤토리 (실측 기준 갱신)

### 4.1 확정 — 소스 코드에서 확인됨

| 이름 | 타입 | 주기 | 소유 노드 | UI 매핑 |
|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 5 Hz | `livox_lidar` | 3D 뷰포트, 측위 입력 |
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz | `livox_imu` | 매핑 엔진 입력 |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | `imu_relay` | EKF 입력 |
| `/scan` | `sensor_msgs/LaserScan` | — | `pointcloud_to_scan` | 2D 지도 레이어 |
| `/Odometry` | `nav_msgs/Odometry` | 10 Hz | FAST-LIO | 로봇 위치, odom 워치독 |
| `/wheel_odom` | `nav_msgs/Odometry` | 10 Hz | `mcu_bridge` | EKF 입력 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | on-demand | `teaser_fpfh_localizer` / UI | 수동 초기위치 |
| ~~`/sc_candidates`~~ | — | — | — | **폐기** — Scan Context 제거로 발행자 없음. allowlist에서도 제거 |
| `/icp_result` | (icp_relocalization) | on-demand | `icp_node` | 측위 수렴 판정 |
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | Nav2 → arbiter | 자율 소스 |
| `/drive_mode` | `std_msgs/String` | on-demand | → arbiter | 자율 모드 |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | 20 Hz | `keyboard_teleop` → arbiter | **웹 데드맨 출력 예정** |
| `/drive_mode_teleop` | `std_msgs/String` | on-demand | → arbiter | 수동 모드 선택 |
| `/cmd_vel_mux` | `geometry_msgs/Twist` | 50 Hz | `cmd_arbiter` | (내부) |
| `/drive_mode_mux` | `std_msgs/String` | 50 Hz | `cmd_arbiter` | (내부) |
| `/cmd_arbiter/owner` | `std_msgs/String` (latched) | 변화 시 | `cmd_arbiter` | **HUD 동작권 표시 (신규 필요)** |
| `/cmd_arbiter/set_owner` | `alm_msgs/srv/SetControlOwner` | on-demand | `cmd_arbiter` | `enterManual()`/`exitManual()` |
| `/emergency_stop` | `std_msgs/Bool` | on-demand | → `command_manager` | E-STOP 버튼 |
| `/mcu/command` | `alm_msgs/McuCommand` | 50 Hz | `command_manager` | (내부) |
| `/mcu/state` | `alm_msgs/McuState` | 10 Hz | `mcu_bridge` | 수동주행 텔레메트리 전체, 배터리, fault |
| `/drive_mode/effective` | `std_msgs/String` | 50 Hz | `command_manager` | 실제 적용 모드 표시 |
| `/joint_states` | `sensor_msgs/JointState` | 10 Hz | `mcu_bridge` | (RViz/URDF) |
| `/map_save` | `std_srvs/srv/Trigger` | on-demand | FAST-LIO | "저장" 버튼 |

### 4.2 미확정 · 신규 필요

| 이름 | 타입 | UI 매핑 | 상태 |
|---|---|---|---|
| 등록 점군 (`/cloud_registered` 추정) | `PointCloud2` | 3D 뷰포트 누적 맵 | **실기에서 토픽명 확정 필요** |
| `teaser_fpfh_localizer` 진행 상태 | 신규 토픽 | 측위 상태 패널 | 노드에 발행 추가 or UI 단순화 |
| `pcd2pgm` 실행 | 서비스 아님 (CLI) | "변환" 버튼 | backend subprocess 래핑 |
| `sc_build_db` 실행 | 서비스 아님 (CLI) | "생성" 버튼 | backend subprocess 래핑 |
| `FollowWaypoints` | `nav2_msgs/action` | 자율주행 시작/진행률 | Nav2 기동 확인 필요 |
| `/global_costmap/costmap`, `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 코스트맵 레이어 | Nav2 기동 확인 필요 |
| `/plan` | `nav_msgs/Path` | 경로 레이어 | Nav2 기동 확인 필요 |
| E-STOP 래치 해제 | 신규 서비스 | 해제 확인 모달 | §3.6 결정에 따름 |
| Jetson 리소스 통계 | 커스텀 토픽 | 모니터링 탭 | 신규 노드 필요 |
| TF `map→odom→base_link` | tf2 | 로봇 마커, 좌표 변환 | `transform_publisher` 존재, 검증 필요 |

**`alm_msgs`에 현재 존재하는 인터페이스는 `msg/McuCommand`, `msg/McuState`, `srv/SetControlOwner` 셋뿐입니다.** `/alm/safety_status`용 메시지는 아직 없음.

---

## 5. 브라우저(WebUI) 측 체크리스트

### 5.1 연동 레이어 아키텍처
- [ ] 신규 모듈 분리: `assets/ros-bridge.js` (Foxglove WS 클라이언트 + 메시지 디코딩), `assets/renderer3d.js` (WebGL 렌더러) — 지금의 `motion.js`/`guide.js`처럼 `app.js`를 건드리지 않는 애드온 패턴을 유지하되, 상태 흐름 자체가 바뀌므로 100% 무수정은 어려움
- [ ] `app.js`의 `state` 객체를 "로컬에서 계산하는 값"이 아니라 "서버가 보내준 마지막 값의 캐시"로 역할 전환 — `setInterval`로 진행률을 만들어내는 함수들을 전부 구독 콜백으로 교체

### 5.2 빌드 도구 도입 여부 (초반에 반드시 결정)
- [ ] **결정 A — 무의존성 유지**: WebSocket 프레임 파싱, CDR 디코딩, WebGL 렌더링을 직접 구현. `index.html`을 그냥 열면 동작, 오프라인 배포에 유리, 개발 공수 증가
- [ ] **결정 B — 번들러 도입**(Vite 등): `@foxglove/ws-protocol`, `@foxglove/schemas`, `three` 사용. 개발 속도는 빠르지만 빌드 스텝이 생김
- [ ] 이 결정에 따라 이후 코드 구조가 갈리므로 Phase 0에서 가장 먼저 확정

### 5.3 PointCloud2 디코딩 및 3D 렌더러
- [ ] PointCloud2 바이너리 필드 파싱(offset, datatype, endianness) 구현 또는 라이브러리 사용
- [ ] `drawPointCloud()` ([assets/app.js:858](assets/app.js#L858)) 전체 교체 — Canvas 2D → WebGL
- [ ] `뷰 리셋`/`탑다운` 버튼이 지금은 토스트만 띄움 ([assets/app.js:947-948](assets/app.js#L947-L948)) — 실제 카메라 조작으로 교체
- [ ] 대용량 포인트 대응: LOD, 포인트 드롭, GPU 인스턴싱
- [ ] **현재 `drawPointCloud()`는 탭과 무관하게 매 프레임 1700 포인트를 그립니다.** 실연동 전에 비활성 탭에서 rAF를 멈추도록 수정 — Jetson CPU 예산과 직접 충돌

### 5.4 2D 지도/코스트맵 렌더링
- [x] 2D 지도는 `/map`(OccupancyGrid) → 캔버스로 교체 완료. 맵 자산은 `maps/<맵이름>/`(§0 참조)에 있고, 존재 여부는 `/alm/map_inventory`가 알려준다
- [ ] OccupancyGrid → 캔버스/이미지 변환 로직 (코스트맵 실시간 갱신)
- [ ] 좌표 변환 함수(`svgPoint` [assets/app.js:425](assets/app.js#L425), `pixelToMap` [assets/app.js:432](assets/app.js#L432))를 실제 지도 `resolution`·`origin`에 맞게 일반화 — 지금은 900×620 뷰박스에 `* 0.02` 하드코딩

### 5.5 TF 처리
- [ ] `map → odom → base_link` 변환을 받아 로봇 마커/카메라 좌표계에 적용 (전체 tf2까지는 불필요, 3개 프레임 수동 처리로 충분)

### 5.6 명령 전송 경로 (안전 최우선) — 개정
- [ ] 데드맨 20 Hz 명령(`startManualCommand` [assets/app.js:742](assets/app.js#L742))을 **`/cmd_vel_teleop` 발행**으로 교체. 경로: **브라우저 → alm_web_backend → `/cmd_vel_teleop` → cmd_arbiter → command_manager → mcu_bridge**
- [ ] backend는 twist만 보낼 것. **4WIS 변환·속도 제한을 브라우저나 backend에서 다시 하지 말 것** — `command_manager`가 유일한 변환·게이팅 지점
- [ ] 기존 클라이언트 안전장치(`blur`/`visibilitychange` 정지, [app.js:998-999](assets/app.js#L998-L999))는 유지하되, **최종 안전 책임은 `cmd_arbiter`의 `teleop_timeout_sec`(0.5s)과 `command_manager`의 `cmd_timeout_sec`(0.5s)에 있음**을 코드 주석·문서에 명시
- [ ] **데드맨 패드가 키보드로 조작 불가** — `.drive-button`이 pointer 이벤트 전용이고 `setPointerCapture` 때문에 `pointerleave` 폴백도 죽어 있음([app.js:992-997](assets/app.js#L992-L997)). 접근성이자 안전 요건이므로 Phase 5 전 수정
- [ ] E-STOP은 `/emergency_stop` 발행 + 상태 확인 후 UI 갱신. **낙관적으로 "해제됨"을 먼저 표시하지 않을 것**
- [ ] `teleop(held)` 상태 표시 UI 추가 (§3.5)

### 5.7 상태 동기화 전략
- [ ] "누가 진실의 원천인가" 결정 — mock에서는 브라우저 `state`가 진실이었지만, 실 연동에서는 **로봇/백엔드가 유일한 진실**
- [ ] 다중 클라이언트 동시 접속 시 상태 브로드캐스트 방식 확정. §2.1의 두 축을 각각 브로드캐스트
- [ ] **탭 잠금이 실제로 막지 않음** — `switchTab()`([app.js:131](assets/app.js#L131))이 매핑 중 경고 토스트만 띄우고 그대로 전환시킴. 실연동 시 잠금이 의미를 갖도록 수정
- [ ] 설정 드로어 프로필 카드가 `state.profile`과 동기화되지 않음(`openSettings()` [app.js:901](assets/app.js#L901)) — 서버 상태를 받아 렌더하도록 교체

---

## 6. app.js 함수별 마이그레이션 매핑표 (개정)

### 01 · 매핑

| 현재 목업 함수 | 실제로 필요한 것 |
|---|---|
| `startMapping()` | backend에 launch 요청 → `lidar.launch.py` + `slam.launch.py` 기동 → 진행 상황은 노드/토픽 활성 여부로 판정 |
| `stopMapping()` | launch 그룹 역순 종료 요청 |
| `savePcd()` | `/map_save` 서비스 호출. **저장 경로가 config 고정이므로 backend가 맵 이름 디렉터리로 이동** |
| `openPcd2Pgm()` | `pcd2pgm.py`를 backend subprocess로 실행, stdout 파싱으로 진행 표시. 파라미터명을 스크립트 인자와 일치시킬 것 |
| `buildScDb()` | `sc_build_db.py`를 backend subprocess로 실행, self-test 결과는 stdout에서 파싱 |
| `state.maps` 배열 | 백엔드 API(`GET /api/maps`)로 `alm_navigation/maps/` 실제 스캔, hash·revision도 서버 계산값 |
| `openNewMapModal()` 이름 검증 | 클라이언트 검증(`^[a-zA-Z0-9_-]{2,32}$`)에 더해 서버에서도 중복 확인 |

### 02 · 측위 · 자율주행

| 현재 목업 함수 | 실제로 필요한 것 |
|---|---|
| `autoLocalization()` | backend가 `teaser_fpfh_localizer` 기동 → 진행 상태 구독(발행 추가 시) 또는 `/icp_result` 수신으로 성공 판정. **일회성 노드이므로 재시도 = 재기동** |
| `manualInitialPose()` + `mapClick()` | 클릭 좌표를 `/initialpose`로 발행 |
| `relocalize()` | `teaser_fpfh_localizer` + `icp_node` 재기동 (둘 다 성공 시 자동 종료되므로) |
| `startNavigation()` | `FollowWaypoints` 액션 목표 전송, 피드백으로 진행률/남은 거리 갱신 |
| `pauseNavigation()`/`cancelNavigation()` | 액션 취소(cancel goal) 호출 |
| 반복 횟수·순환 주행 옵션 | Nav2 자체 기능이 아니므로 backend가 액션을 반복 호출 |
| 측위 후보 표시 | `/sc_candidates`(PoseArray) 구독 |
| 지도 레이어(코스트맵 등) | 대응 토픽 구독 후 캔버스/이미지로 렌더링 |

### 03 · 수동주행 (개정)

| 현재 목업 함수 | 실제로 필요한 것 |
|---|---|
| `enterManual()` | **`set_owner("teleop")` 서비스 호출** + 모터 활성 확인. 응답의 `active_owner`로 UI 갱신 |
| `exitManual()` | **`set_owner("auto")` 서비스 호출** |
| `commandFor()` + `startManualCommand()` | 계산한 Twist를 backend 경유로 `/cmd_vel_teleop` 발행. **속도 상수는 서버에서 받아올 것**(§12) |
| 주행 모드 버튼(normal/spin/crab/auto) | `/drive_mode_teleop` 발행. `mode_id` 매핑: 0=정지 1=일반 2=자가추종(미구현) 3=크랩 4=제로턴 |
| Crab 비활성 안내 | `command_manager`의 `auto_crab_enabled` 파라미터를 읽어 동적 판정 (지금은 문자열 하드코딩) |
| `updateManualTelemetry()` ([assets/app.js:760](assets/app.js#L760)) | `/mcu/state`의 `measured_velocity`, `wheel_speed[4]`, `steer_angle[2]` 구독으로 교체. **조향각은 4개가 아니라 2개** |
| 배터리 전압/전류 | `/mcu/state`의 `battery_voltage`/`battery_current` |
| (신규) 동작권 상태 표시 | `/cmd_arbiter/owner` 구독 — `auto` / `teleop` / `teleop(held)` |
| (신규) 실제 적용 모드 | `/drive_mode/effective` 구독 — auto 모드가 내부적으로 뭘 골랐는지 표시 |

### 04 · 모니터링

| 현재 목업 함수 | 실제로 필요한 것 |
|---|---|
| `updateMetrics()` | Jetson 리소스 통계 토픽 구독 |
| ROS 2 그래프 목록 | 실제 토픽 활성 여부(주기 계산 포함)를 backend가 조회해 전달 |
| `refreshProcesses` | backend가 systemd/프로세스 상태 조회 |
| 로그 창 | `/rosout` 구독 (노드명·레벨이 이미 UI 포맷과 일치) |
| `exportSnapshot()` | 그대로 유지 가능 — 마지막으로 받은 실데이터를 JSON으로 내보내면 됨 |

---

## 7. 안전 검토 체크리스트 (실 로봇 연동 시 반드시)

- [ ] 물리 비상정지가 웹 UI와 무관하게 항상 최우선으로 동작하는지 실제 케이블 분리 테스트
- [ ] 네트워크 완전 단절 시 로봇이 정지하는지 실측 — **`cmd_arbiter` 0.5s → `teleop(held)`**, **`command_manager` cmd_timeout 0.5s → 하드 정지**. 두 계층이 모두 도는지 확인
- [ ] 브라우저 탭 강제 종료(kill) 시 데드맨 정지 확인 — JS의 `blur`/`visibilitychange`가 못 잡는 크래시 상황
- [ ] **`teleop(held)` 진입 후 자율이 임의로 재개되지 않는지 확인** — 설계상 `set_owner("auto")` 명시 호출로만 복귀해야 함
- [ ] **웹 텔레옵과 `keyboard_teleop` 동시 실행 시 거동 확인** (§3.5의 결정 (a)/(b)에 따라)
- [ ] 다수의 관전자가 동시 접속했을 때 웹 세션 제어권이 한 명에게만 있는지 **서버 측에서도** 강제되는지 확인
- [ ] 상태 전이 중(매핑 진행 중 새 맵 생성 시도 등) 명령이 겹치면 어떻게 되는지 경쟁 상태 테스트
- [ ] `base_control.yaml`의 `##CONFIRM##` 4WIS 파라미터를 실차로 확정한 뒤에만 웹 수동주행 테스트
- [ ] 실제 로봇 주변에 사람이 없는 통제된 공간에서, **잭업 상태 → 저속(속도 배율 25%)** 순으로 첫 테스트

---

## 8. 보안 체크리스트

- [ ] **innerHTML 이스케이프 문제 — 연동 전 필수 수정.** `toast()`([app.js:72](assets/app.js#L72)), `renderLogs()`([app.js:109](assets/app.js#L109)), `renderWaypoints()`, `openModal()`이 문자열을 이스케이프 없이 삽입. **`/rosout`을 로그 창에 연결하는 순간 XSS 경로가 됩니다** — 노드가 뱉는 `fault_text`도 마찬가지. 외부에서 오는 문자열은 `textContent` 기반으로 교체
- [ ] Foxglove WSS 인증(토큰/비밀번호), TLS 인증서 유효성 확인
- [ ] Foxglove allowlist에서 `/cmd_vel*`, `/mcu/command`, `/emergency_stop` **쓰기 경로 제외** (§3.8)
- [ ] `alm_web_backend` REST API 인증 — 같은 네트워크라도 최소한의 토큰 인증 권장
- [ ] 로봇이 있는 네트워크와 외부 인터넷 분리(VPN/방화벽)
- [ ] **웹폰트 CDN 의존 제거** — `index.html:10,14`가 jsdelivr/Google Fonts를 부름. 오프라인 온보드 배포 시 타이포가 깨지고, 외부 요청 자체가 격리 정책 위반이 될 수 있음. `assets/fonts/`로 내려받아 로컬 참조로 교체

---

## 9. 단계별 롤아웃 로드맵 (개정)

| Phase | 내용 | 완료 기준 |
|---|---|---|
| **0. 준비** | 빌드 도구 결정(§5.2), 등록 점군 토픽명 확정, `fast_lio` 설치 위치 확인, E-STOP 래치 방식 결정(§3.6), 웹/키보드 텔레옵 경쟁 해소 방식 결정(§3.5), **XSS 이스케이프 수정**, 비활성 탭 rAF 정지, 폰트 로컬화 | §4.2 "미확정" 표에 확인 필요 항목이 남지 않음 |
| **1. 읽기 전용 연동** | foxglove_bridge 도입. 모니터링 탭 + `/mcu/state` + `/cmd_arbiter/owner` + `/drive_mode/effective` + `/rosout` 표시. 조작은 여전히 mock | 모니터링 탭 수치가 실측값과 일치, HUD 동작권 배지가 실제 owner를 따라감 |
| **2. 시각화 연동** | 포인트클라우드/지도/로봇 위치/코스트맵을 실데이터로 표시 (뷰어 모드) | 실제 SLAM 세션에서 화면이 RViz와 동일한 위치·형태로 갱신됨 |
| **3. 저위험 명령 연동** ✅ **완료 (2026-08-10)** | `alm_web_backend` 신설. E-STOP 래치, 웹 세션 제어권, 맵 저장(`/map_save`), `pcd2pgm`·`fpfh_map_builder` subprocess, 맵 폴더 생성·활성 전환, SLAM launch 기동·종료 | 버튼 클릭만으로 실제 grid.pgm / fpfh_map* 이 생성됨을 확인. `cloud.pcd` 생성(=`/map_save`)만 실센서 필요 |
| **4. 측위/자율주행 명령 연동** | `teaser_fpfh_localizer` 기동/재기동, `/initialpose` 발행, `FollowWaypoints` 액션 | 실제 로봇이 웹 UI로 지정한 웨이포인트를 따라 이동 |
| **5. 수동주행(데드맨) 연동** | `set_owner` + `/cmd_vel_teleop` 발행. **가장 안전 critical — 반드시 마지막.** 선행 조건: `base_control.yaml`의 `##CONFIRM##` 확정, 데드맨 키보드 지원, `teleop(held)` UI | §7 전항목 통과 + 잭업 검증 + 통제 환경 저속 검증 후에만 배포 |

각 Phase 종료 시 §7 안전 체크리스트를 재확인합니다.

**개정 메모**: 명령 체인이 이미 있으므로 Phase 5의 *구현* 난이도는 초판 예상보다 낮습니다(붙일 곳은 `/cmd_vel_teleop` 하나). 그러나 *위험도*는 그대로이고, 4WIS 변환 상수가 미확정인 상태에서는 twist가 그대로 잘못된 rpm이 되므로 순서는 바꾸지 않습니다.

---

## 10. 테스트 · 검증 체크리스트

- [ ] Phase별 통합 테스트 시나리오 문서화
- [ ] 네트워크 지연/끊김 시뮬레이션 테스트(`tc netem`으로 인위적 지연 주입)
- [ ] 다중 브라우저 탭 동시 접속 테스트(웹 세션 제어권 충돌 확인)
- [ ] 장시간 연속 운용 테스트 — 메모리 누수, WebSocket 재연결, 포인트클라우드 렌더러 프레임 드랍
- [ ] 반응형 레이아웃(390px 포함)에서 실데이터 렌더링도 깨지지 않는지 재확인
- [ ] **`ros2 topic hz`로 UI 표기 주기(5.0 Hz, 10 Hz, 50 Hz)와 실측 일치 확인**

---

## 11. 열린 질문 / 다음에 정해야 할 것

- [x] ~~데드맨 명령의 검증 계층을 어디에 둘지~~ → **`cmd_arbiter` + `command_manager`가 이미 담당. backend는 얇게 유지** (2026-08-06 해결)
- [x] ~~`/mcu/state` 메시지 필드 정의~~ → **`alm_msgs/msg/McuState`로 확정됨** (2026-08-06 해결)
- [ ] `/alm/safety_status`를 별도 메시지로 만들지, `/mcu/state` + `/cmd_arbiter/owner` 조합으로 대신할지
- [x] ~~E-STOP 래치를 `command_manager`에 넣을지 backend에 둘지~~ → **`command_manager` 에 넣음** (`estop_latch: true` + `alm_msgs/srv/ReleaseEstop`). backend 가 죽어도 정지가 유지되어야 하므로 (2026-08-10 해결)
- [ ] 웹 텔레옵을 `/cmd_vel_teleop` 공유로 갈지, arbiter에 `web` owner를 추가할지 (§3.5) — **Phase 5 착수 전 필수**
- [ ] FAST-LIO의 정확한 등록 점군 출력 토픽명 (실기 `ros2 topic list` 필요)
- [ ] `fast_lio` 패키지의 설치 위치 — 이 워크스페이스 의존성으로 명시할지
- [ ] `/map_save`의 맵 이름별 경로 처리 방식 (config 파라미터 변경 vs 저장 후 이동)
- [ ] `teaser_fpfh_localizer`에 진행 상태 발행을 추가할지, UI를 3단계로 단순화할지
- [ ] 웨이포인트 반복/순환을 backend가 반복 호출로 구현 (Nav2 기본 기능 아님 — 확정)
- [ ] §5.2 빌드 도구 도입 여부 최종 결정
- [ ] 원격(외부 네트워크) 접속 지원 여부

---

## 12. UI ↔ 실물 불일치 목록 (개정 신설)

연동 작업 중 반드시 하나씩 해소해야 하는, **화면과 로봇이 서로 다른 것을 뜻하고 있는** 지점들입니다.

| # | UI가 전제하는 것 | 실물 | 해소 방향 |
|---|---|---|---|
| 1 | `command_gateway` 단일 노드 | `cmd_arbiter` + `command_manager` + `mcu_bridge` | UI 로그 문구·프로세스 목록을 실물 이름으로 교체 |
| 2 | `safety_supervisor`가 안전을 판단 | `command_manager` 안에 게이팅이 들어 있음 | 별도 노드 만들지 말 것. 집계 발행만 검토 |
| 3 | ~~E-STOP이 래치 (문구 입력으로 해제)~~ | ~~`command_manager`는 토픽 값 추종~~ | ✅ **해소 (2026-08-10)** — `estop_latch: true`. 토픽 `false` 는 무시, 해제는 `/emergency_stop/release` 서비스로만. MCU fault 유지 중이면 거부 |
| 4 | ~~제어권 = 단일 개념~~ | 웹 세션 제어권 / 동작권 2축 | ✅ **해소 (2026-08-10)** — 웹 세션 락은 `alm_web_backend`(리스 15초), 동작권은 `cmd_arbiter` 로 분리 |
| 5 | 동작권 상태 2가지 (보유/관전) | `auto` / `teleop` / **`teleop(held)`** | `held` 표시 UI 신규 추가 |
| 6 | 바퀴 4개 조향각 개별 표시 | `steer_angle[2]` (앞축/뒤축) | UI를 축 단위로 수정 |
| 7 | ~~속도 한계가 JS에 하드코딩~~ | `command_manager`의 파라미터와 **우연히 일치**했을 뿐 | ✅ **해소 (2026-08-10)** — `GET /api/limits` 가 `command_manager` 의 실제 파라미터를 읽어 내려주고, `commandFor()` 와 설정 드로어가 그 값을 쓴다 |
| 8 | Crab 비활성 = 고정 문구 | `auto_crab_enabled: false` 파라미터 | 파라미터 조회로 동적 판정 |
| 9 | "감독 하트비트 5 Hz" | 그런 개념 없음. cmd timeout 0.5s가 실질 감시 | UI 문구 수정 또는 하트비트 구현 |
| 10 | `pcd2pgm`/`sc_build_db`가 서비스 | argparse CLI 스크립트 | backend subprocess 래핑 |
| 11 | `/map_save`가 맵 이름별 경로에 저장 | config 고정 경로 | backend가 이동 또는 파라미터 변경 |
| 12 | 측위가 단계별 진행률을 준다 | `teaser_fpfh_localizer`는 상태 미발행 + 성공 시 자체 종료 | 발행 추가 or UI 단순화. "재측위 = 재기동" 반영 |
| 13 | 매핑 중 탭 잠금이 동작 | `switchTab()`이 토스트만 띄우고 전환됨 | 실제 차단으로 수정 |
| 14 | 데드맨을 키보드로도 조작 가능 | pointer 전용, `pointerleave` 폴백도 무력 | Phase 5 전 필수 수정 |

---

## 부록 A. 참조 문서

- [`BUTTON_COMMAND_MAP.md`](BUTTON_COMMAND_MAP.md) — **UI 버튼 단위로 Jetson에서 실행되는 명령 매핑** (이 문서의 실행 상세판)
- [`docs/control_arbitration.md`](../docs/control_arbitration.md) — 명령 경로와 동작권 중재 설계 (실물 기준)
- [`ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md`](../ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md) — STM32 4WIS UART 규격 v2
- [`ALM_auto_ws/src/alm_base_control/config/base_control.yaml`](../ALM_auto_ws/src/alm_base_control/config/base_control.yaml) — arbiter/command_manager 파라미터 (`##CONFIRM##` 항목 확인처)
- [`README.md`](README.md) — WebUI v0.6 디자인 규약 및 알려진 코드 이슈
