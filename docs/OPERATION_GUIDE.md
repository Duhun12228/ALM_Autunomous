# ALM 자율주행 운영 가이드

현재 워크스페이스 기준 운영 순서입니다. 이 프로젝트는 기존 2D
`slam_toolbox + AMCL + EKF` 흐름에서 벗어나, **FAST-LIO2 3D 매핑**과
**FAST-LIO-Localization 측위**를 사용합니다.

핵심 실행 경로:

```text
Livox MID-360 UDP 직접 파싱
  -> /livox/lidar, /livox/imu, /scan
FAST-LIO2 매핑
  -> maps/<맵이름>/cloud.pcd
pcd2pgm
  -> maps/<맵이름>/grid.pgm + grid.yaml
fpfh_map_builder
  -> maps/<맵이름>/fpfh_map*  (FPFH+TEASER++ 초기측위 DB)
FAST-LIO-Localization
  -> map->odom, odom->base_link, /Odometry
Nav2
  -> /cmd_vel
command_manager + mcu_bridge
  -> /mcu/command -> STM32
```

## 0. 센서/TF 기준

| 용도 | 입력/출력 | 담당 |
|---|---|---|
| 3D 점군 | `/livox/lidar` | `alm_sensors/scripts/livox_udp_pointcloud2.py` |
| 내장 IMU | `/livox/imu` | `alm_sensors/scripts/livox_udp_imu.py` |
| EKF용 IMU relay | `/imu/data` | `imu_relay.py` |
| 2D costmap용 scan | `/scan` | `pointcloud_to_scan.py` |
| 3D 매핑 | `/livox/lidar` + `/livox/imu` -> PCD | FAST-LIO2 |
| 재측위 | prior PCD + 현재 scan -> `map->odom` | ICP + transform_publisher |
| 실시간 추적 | LiDAR/IMU -> `odom->base_link`, `/Odometry` | FAST-LIO |
| 주행 명령 | `/cmd_vel` -> `/mcu/command` | command_manager |

주행 모드에서는 FAST-LIO-Localization이 TF를 담당하므로 EKF를 끕니다.
매핑 모드에서는 `robot.launch.py` 기본값 때문에 EKF가 켜질 수 있지만,
맵 자체는 LiDAR+IMU 기반 FAST-LIO 결과이며 엔코더는 맵 생성에 직접 관여하지 않습니다.

## 1. 하드웨어 기본 스택 확인

```bash
cd ~/ALM_Autunomous/ALM_auto_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch alm_bringup robot.launch.py
```

다른 터미널에서 확인:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /scan
ros2 topic hz /mcu/state
ros2 topic hz /wheel_odom
ros2 run tf2_tools view_frames
```

`/scan`이 비어 있으면 LiDAR 장착 높이와 `pointcloud_to_scan`의
`min_height/max_height`를 조정합니다.

## 2. 3D 매핑

센서만 먼저 올리거나, 상위 bringup의 slam launch를 사용합니다.

```bash
cd ~/ALM_Autunomous/ALM_auto_ws
source install/setup.bash

ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation slam.launch.py rviz:=true
```

또는 통합 launch:

```bash
ros2 launch alm_bringup slam.launch.py
```

로봇을 천천히 움직이며 공간 전체를 훑습니다. 같은 구간으로 되돌아오면
드리프트 확인이 쉽습니다. 매핑 RViz의 Fixed Frame은 `odom`입니다.

맵 저장:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

기본 저장 경로:

```text
~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/alm_lab/cloud.pcd
```

PCD는 대용량 로컬 산출물이므로 `.gitignore`에서 제외됩니다.

## 3. 3D PCD를 Nav2용 2D 맵으로 변환

Nav2의 global costmap static layer에는 2D occupancy map이 필요합니다.
FAST-LIO가 만든 PCD를 `pcd2pgm.py`로 변환합니다.

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAP=$WS/src/alm_navigation/maps/alm_lab      # 맵 하나 = 폴더 하나

ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAP/cloud.pcd \
  --out $MAP/grid \
  --resolution 0.05 \
  --z-min 0.3 \
  --z-max 0.8
```

출력:

```text
maps/alm_lab/grid.pgm
maps/alm_lab/grid.yaml
```

`pcd2pgm.py`가 출력하는 z 분포를 보고 벽/장애물만 잡히도록
`--z-min`, `--z-max`를 조정합니다.

## 4. 측위만 검증

```bash
WS=~/ALM_Autunomous/ALM_auto_ws

ros2 launch alm_sensors lidar.launch.py
# 인자를 비우면 maps/active.yaml 이 가리키는 맵을 자동으로 쓴다
ros2 launch alm_navigation localization.launch.py
```

2D 맵을 RViz에 함께 띄워 확인:

```bash
# 인자를 비우면 active.yaml 을 따라간다 (맵을 바꾸면 화면도 따라온다)
ros2 run alm_navigation map_publisher.py
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz
```

RViz에 `/teaser_aligned_cloud`(PointCloud2)를 추가하면 추정 pose로 변환한 현재
스캔이 맵 프레임에 그려집니다 — 맵과 겹치는지로 정합 품질을 눈으로 봅니다.

**초기위치를 지정할 필요가 없습니다.** FPFH+TEASER++는 초기 추정 없이 맵 전체에서
찾습니다(전역 정합). RViz의 2D Pose Estimate는 이 브랜치에서 아무도 구독하지
않습니다 — `/initialpose`를 받는 것은 쓰지 않는 `icp_node`입니다.

띄운 뒤 **로봇을 정지 상태로 두세요.** 정합 전에 라이다 프레임을
`accum_frames`(기본 10)만큼 누적합니다.

성공 판정:

```bash
ros2 run tf2_ros tf2_echo map odom   # 이게 나오면 정합 성공
ros2 topic hz /Odometry              # ~10Hz 추적 중
ros2 topic echo /icp_result --once   # 확정된 초기 pose
```

`teaser_fpfh_localizer`는 성공해도 **죽지 않고 유휴 상태로 남습니다**
(`finished_` 플래그). 노드가 살아 있는 것이 정상입니다. 대신 리셋 기능이 없어서
다시 찾게 하려면 launch를 재기동해야 하고, 그러면 같은 launch의 FAST-LIO도
함께 재시작되어 **추적 오도메트리가 초기화됩니다.**

로그 태그로 어디서 걸리는지 읽습니다:

| 로그 | 뜻 |
|---|---|
| `[ACCUM] frame n/N` | 스캔 누적 중 — 로봇을 세워 두세요 |
| `[FPFH] raw=… features=…` | 현재 스캔에서 특징 추출 |
| `[MATCH] matches=9 (required >= 20)` | DB와 대응점 부족 — **가장 흔한 실패** |
| `[TEASER] … overlap=… rmse=…` | 전역 정합 결과와 검증 수치 |
| `[TEASER] rejected by … validation` | 정합은 됐으나 겹침률/RMSE 미달 (오정합 차단) |
| `[GICP] did not converge` | 지역 정밀화 실패 |
| `[CONSISTENCY] accepted 1/2` | 한 번 성공, 한 번 더 같은 답을 기다리는 중 |
| `[ATTEMPT n] rejected` | 그 시도 포기, 새 스캔 누적으로 되돌아감 |
| `localization succeeded; /icp_result published` | **성공** |

`[MATCH]`에서 계속 걸리면 측위 DB의 feature가 부족한 경우가 많습니다.
`fpfh_map.meta`의 `feature_count`를 확인하고, 1000개 미만이면 `voxel`을 줄여
DB를 다시 만드세요. **DB 생성 파라미터와 `localization.launch.py`의
`feature_voxel`/`normal_radius`/`feature_radius`는 반드시 같아야 합니다.**

### 맵 지정은 한 곳에서만

`map_pcd` 하나가 `teaser_fpfh_localizer`와 `fast_lio` **양쪽에 동시에** 들어갑니다
(`prior_map_path`를 launch가 덮어씁니다). 그래서 둘이 서로 다른 맵을 보는 구성이
애초에 만들어지지 않습니다. `fastlio_relocalization.yaml`의 `prior_map_path`는 이
launch 없이 `fast_lio`를 직접 띄울 때의 기본값일 뿐입니다.

## 5. 자율주행

```bash
# 인자 없이 실행하면 maps/active.yaml 이 가리키는 맵을 쓴다
ros2 launch alm_bringup navigation.launch.py
```

다른 맵을 쓰려면 `maps/active.yaml` 의 `active:` 를 바꾸거나 인자로 덮어씁니다:

```bash
MAP=~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/<맵이름>

ros2 launch alm_bringup navigation.launch.py \
  map:=$MAP/grid.yaml \
  map_pcd:=$MAP/cloud.pcd \
  fpfh_db_prefix:=$MAP/fpfh_map
```

이 launch는 다음을 함께 올립니다.

- `robot.launch.py use_ekf:=false`
- `nav2_map_server`
- FAST-LIO-Localization
- Nav2 planner/controller/BT

초기 위치는 지정하지 않습니다 — FPFH+TEASER++가 자동으로 찾습니다(§4).
`map → odom` TF가 나오기 전까지는 목표를 줘도 의미가 없으니 먼저 확인하세요.

### RViz에서 웨이포인트 찍기

```bash
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/navigation.rviz
```

`localization.rviz` 와 달리 **Navigation 2 패널**(`nav2_rviz_plugins`)과
코스트맵·경로 디스플레이가 들어 있습니다. 웨이포인트 모드가 그 패널에 있습니다.

1. 왼쪽 아래 **Navigation 2** 패널에서 **Waypoint / Nav Through Poses mode** 클릭
2. 상단 툴바 **Nav2 Goal** 로 맵을 찍습니다. 드래그해서 놓으면 **놓는 방향이 그 점의 yaw** 입니다
3. 원하는 만큼 반복 — 찍을 때마다 목록에 쌓입니다
4. **Start Waypoint Following** 클릭

패널 버튼은 `/follow_waypoints` 액션을 부릅니다. CLI 로 같은 것을 보내려면:

```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}]}" --feedback
```

한 점만 갈 때는 패널 조작 없이 **Nav2 Goal** 을 바로 찍으면 됩니다
(`/goal_pose` → `navigate_to_pose`).

Nav2는 `/cmd_vel`을 만들고, `command_manager`가 `auto` 모드에서
`normal/spin/crab` 중 실제 MCU에 보낼 모드를 선택합니다. 현재 crab은 기본 비활성입니다.

⚠ **실차 주행 전에 `/mcu/command` 를 먼저 보세요.** `base_control.yaml` 의 4WIS
변환 상수 8개가 아직 `##CONFIRM##` 입니다. Nav2 의 twist 가 그 상수를 거쳐
rpm·조향각이 되므로, 값이 틀리면 명령과 다른 방향으로 갑니다. 바퀴를 띄운
상태에서 `/mcu/command` 가 상식적인 값인지 확인한 뒤 바닥에 내리세요.

## 6. 주행 모드

`/drive_mode`는 `std_msgs/String`입니다.

```bash
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

모드:

- `normal`: 전후진 + 회전
- `spin`: 제자리 회전
- `crab`: 측면 병진, 기본 자동 선택 비활성
- `auto`: `/cmd_vel`을 보고 normal/spin을 자동 전환

실제로 적용된 모드는 `/drive_mode/effective`에서 확인합니다.

## 6-1. WebUI 관제 콘솔

```bash
# 1) 명령 경로용 토큰 (없으면 백엔드가 기동을 거부합니다)
export ALM_WEB_TOKEN=$(openssl rand -hex 16)
echo $ALM_WEB_TOKEN          # 브라우저 설정에 넣을 값

# 2) 스택 (읽기 브리지 + 쓰기 백엔드 + 더미 센서/MCU)
ros2 launch alm_bringup webui_dev.launch.py

# 3) 정적 파일 서버
cd alm-webui-v0.6 && npm run build && npm run serve   # :8080
```

브라우저에서 `http://<젯슨IP>:8080` → **설정** 드로어에 백엔드 주소(`http://<젯슨IP>:8081`)와
토큰을 넣습니다. 토큰은 `sessionStorage` 라 탭을 닫으면 지워집니다.

포트가 둘인 이유 — **읽기와 쓰기가 서로 다른 경로**입니다.

| 포트 | 프로세스 | 방향 | 인증 |
|---|---|---|---|
| 8765 | `foxglove_bridge` | 로봇 → 브라우저 (구독 전용) | 없음. 대신 발행·서비스 호출 능력 자체가 빠져 있음 |
| 8081 | `alm_web_backend` | 브라우저 → 로봇 (명령) | Bearer 토큰 + 웹 제어권 락 |

조작을 하려면 HUD의 **제어권** 버튼으로 락을 잡아야 합니다. 한 번에 한 명만 잡을 수 있고,
브라우저가 죽으면 15초 뒤 자동으로 풀립니다. **E-STOP 만 예외**로 락 없이 누를 수 있습니다.

⚠ **E-STOP 은 래치입니다.** 토픽으로 `false` 를 보내도 안 풀리고, 해제는 서비스로만 됩니다.

```bash
ros2 service call /emergency_stop/release alm_msgs/srv/ReleaseEstop "{reason: 'manual'}"
```

MCU 가 fault/estop 을 보고하는 동안에는 해제가 **거부**됩니다 — 물리 조건을 먼저 푸세요.

### 웹에서 초기위치 자동정합

**측위 · 자율주행** 탭의 **자동 탐색** 버튼입니다. 맵을 고를 필요가 없습니다 —
`maps/active.yaml` 의 활성 맵을 서버가 읽어 경로를 조립하고, `map_pcd` 하나가
`teaser_fpfh_localizer` 와 `fast_lio` 양쪽에 들어갑니다(§4 참조).

기동 **전에** 아래를 확인하고, 하나라도 걸리면 프로세스를 띄우지 않고 이유를
409 로 돌려줍니다. 측위는 실패해도 조용하기 때문에, 몇 분 기다린 뒤 로그를
뒤지게 만들지 않기 위해서입니다.

| 점검 | 걸렸을 때 |
|---|---|
| `cloud.pcd` 존재 | "먼저 매핑하고 저장하세요" |
| FPFH DB 파일 4개 존재 | "'FPFH 측위 DB 생성' 을 먼저 실행하세요" |
| DB 의 `map_path` 가 이 맵인지 | "다른 맵으로 만들어진 DB 입니다" |
| DB 의 점 개수 = 현재 `cloud.pcd` | "DB 를 다시 만드세요" |
| `cloud.pcd` 가 DB 보다 최신인지 | 같음 (재매핑 후 DB 재생성 누락) |
| `/livox/lidar` 발행자 존재 | "먼저 라이다를 켜세요" |
| `slam` 슬롯이 도는지 | "먼저 종료하세요 — FAST-LIO 두 개는 TF 가 겹칩니다" |
| **ROS 그래프에 측위 노드가 이미 있는지** | CLI 로 띄운 것까지 잡습니다 (슬롯 표에는 안 보입니다) |

**정합 로그 패널**이 같은 탭에 있습니다. 출처가 둘이고 잡는 실패가 다릅니다.

| 출처 | 보이는 것 |
|---|---|
| ROS 로그 | `/rosout` 의 `[ACCUM]`/`[MATCH]`/`[TEASER]`/`[GICP]`/`[CONSISTENCY]` — 정상 동작 중의 진행 |
| 프로세스 출력 | launch 의 stdout/stderr — **기동 실패, 맵 로딩 중 크래시, PCL 예외** |

노드가 뜨기도 전에 죽으면 `/rosout` 에는 한 줄도 안 남습니다. **ROS 로그 패널이
비어 있는 것 자체가 증상**일 때 볼 곳이 프로세스 출력입니다.

`[ACCUM]` 은 초당 18줄씩 찍혀서 두 패널 모두에서 걸러냅니다 — 프레임 진행은
진행바가 보여주고, 목록에는 읽어야 할 거절 사유만 남깁니다.

**수동 지정** 버튼은 붙일 대상이 없어 목업으로 남아 있습니다. 이 브랜치의
`teaser_fpfh_localizer` 는 `/initialpose` 를 구독하지 않습니다.

**재측위** 는 슬롯 재기동이라 추적 오도메트리가 초기화됩니다 — 확인창이 뜹니다.

## 6-2. 음성 안내

실차에서는 화면을 못 봅니다. 명령이 로봇까지 갔는지 **귀로** 확인합니다.
블루투스 스피커(`TS-BTS25-2-D`)로 짧은 영어 문장이 나갑니다.

음성은 `voice_announcer` 노드가 전담하며, **systemd 사용자 서비스로 부팅 때부터
자동 기동**합니다. launch 에서 또 띄우지 마세요 (`use_voice` 기본값 `false`).

```bash
scripts/install_voice_service.sh          # 설치 (1회)
sudo loginctl enable-linger $USER         # 로그인 없이도 뜨게 (1회, 필수)
systemctl --user status alm-voice.service
```

### 말하는 것

| 계기 | 접수 즉시 | 완료 시 | 우선순위 |
|---|---|---|---|
| 매핑 시작 | `Starting mapping` | `Mapping started` / `…failed to start` | 1 |
| 매핑 종료 | `Stopping mapping` | `Mapping stopped` | 1 |
| 맵 저장 | `Saving map` | `Map saved` / `Map save failed` | 1 |
| 2D 변환 | `Building two D map` | `Two D map ready` / `…failed` | 1 |
| 측위 DB | `Building localization database` | `…ready` / `…failed` | 1 |
| **측위 시작** | `Starting localization` | `Localization started` / `…failed to start` | 1 |
| **측위 수렴** | — | `Localization converged` | 1 |
| **측위 종료** | — | `Localization stopped` | 1 |
| 프로세스 급사 | — | `S L A M process exited` / `Localization process exited` | 2 |
| **제어권 획득** | — | `Control acquired` | 1 |
| **제어권 반납** | — | `Control released` | 1 |
| **제어권 만료** | — | `Control timed out` | 1 |
| E-STOP | — | `Emergency stop` / `…released` | 2, 선점 |
| MCU fault | — | `M C U fault` | 2, 선점 |
| 스피커 재생 버튼 | — | Wi-Fi 이름 + IP (느리게) | 1 |

**반납과 만료는 다른 문구입니다.** `Control released` 는 조작자가 끝낸 것이고,
`Control timed out` 은 **쥔 채로 연결이 끊긴** 것입니다 — 조작자가 Wi-Fi 범위
밖으로 걸어나가면 15초 뒤에 이 소리가 납니다. 만료는 HTTP 요청이 하나도 없는
채로 일어나므로 백엔드가 2초 주기로 직접 감시합니다(`session.poll`).

안전 사건(E-STOP, MCU fault)은 **웹을 거치지 않습니다.** `voice_announcer` 가
토픽을 직접 구독하므로 백엔드가 죽어 있어도, 물리 버튼으로 눌려도 소리가 납니다.

### 스피커 재생 버튼

블루투스 스피커의 재생 버튼을 누르면 **현재 네트워크 상태**를 읽습니다 —
Wi-Fi 이름과 IP 를 한 자리씩, 조금 느리게. 젯슨 IP 를 모를 때 이걸로 확인합니다.
연결이 없으면 `Wifi not connected` 라고 말합니다.

### 끄기

```bash
ros2 param set /voice_announcer enabled false   # 즉시 (데모 중)
systemctl --user stop alm-voice.service         # 완전히
```

오디오가 실패해도 로봇은 영향받지 않습니다. 모든 발화는 fire-and-forget 이라
스피커를 꺼도 API 응답 시간이 그대로입니다(실측 4 ms 대).

## 7. 자주 보는 문제

| 증상 | 확인 |
|---|---|
| `/livox/lidar` 없음 | Jetson IP `192.168.1.5`, LiDAR IP/포트, UDP 수신 여부 |
| `/livox/imu` 없음 | `MID360_config.json`의 host IMU port `56401`, 네트워크 |
| `/scan` 비어 있음 | LiDAR 높이, `pointcloud_to_scan` 높이 필터 |
| `/mcu/state` 없음 | `/dev/ttyTHS1`, baud `115200`, 권한, STM32 프로토콜 |
| TF 충돌 | 주행 모드에서는 EKF off, FAST-LIO가 `odom->base_link` 담당 |
| Nav2가 odom을 못 봄 | `nav2.yaml`의 `odom_topic`은 `/Odometry` |
| 로봇이 안 움직임 | `/cmd_vel`, `/mcu/command`, `/drive_mode/effective`, e-stop, MCU fault |
| E-STOP 이 안 풀림 | 래치입니다. `/emergency_stop/release` 서비스로만 해제 (§6-1) |
| WebUI 백엔드 점이 노랑 | 토큰 미입력. 설정 드로어에서 `ALM_WEB_TOKEN` 값을 넣으세요 |
| WebUI 조작 버튼이 409 | 제어권 미보유. HUD 제어권 버튼으로 락을 잡으세요 |
| 소리가 두 번 겹쳐 들림 | `use_voice:=true` 로 띄워 systemd 서비스와 노드가 둘이 됐다 |
| 스피커 버튼이 무반응 | `voice_announcer` 가 둘이면 두 번째 BlueZ 등록이 실패한다 (어댑터당 1개) |
| 조작 중 `Control timed out` | 하트비트가 15초 끊겼다 — Wi-Fi 확인. 제어권을 다시 잡으세요 |
| 백엔드가 안 뜸 | `ALM_WEB_TOKEN` 미설정(fail-closed). 로그에 안내가 찍힙니다 |
| 노드가 PermissionError 로 죽음 | `--symlink-install` 은 소스의 실행비트를 그대로 씁니다. `chmod +x` 하세요 |
| 웹 측위가 409 `측위 노드가 이미 떠 있습니다` | CLI 로 먼저 띄운 스택이 있습니다. 그 터미널에서 Ctrl-C 하세요 |
| 웹 측위가 409 `DB 가 짝이 안 맞습니다` | 맵을 다시 저장한 뒤 FPFH DB 를 안 만들었습니다 |
| 측위 로그창이 계속 비어 있음 | 노드가 뜨기 전에 죽으면 `/rosout` 에 한 줄도 안 남습니다 — **프로세스 출력** 탭을 보세요 |
| `[MATCH] matches=…(required >= 20)` 반복 | DB feature 부족. `voxel` 을 줄여 DB 재생성 (§4 참조) |

## 8. 현재 운영상 주의
- 런타임에서는 livox_ros_driver2 노드를 실행하지 않고 UDP 직접 파서를 사용합니다.
- `livox_udp_pointcloud2.py`의 host IP/point port는 현재 상수입니다.
  네트워크를 바꾸면 스크립트 값도 확인해야 합니다.
- Python UDP 파서는 부하가 큽니다. TODO에 적힌 대로 C++ 이식 또는 필터링 튜닝이
  장시간 실주행 전 우선 과제입니다.
