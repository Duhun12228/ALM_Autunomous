# ALM Autonomous

Livox MID-360과 Jetson, STM32를 사용하는 ALM 4WIS(4-Wheel Independent Steering)
자율주행 플랫폼의 ROS 2 스택입니다. 현재 `main`이 현행 기준 브랜치이며,
FAST-LIO2 매핑부터 FPFH+TEASER++ 자동 초기측위, Nav2 경로계획, STM32 구동,
WebUI 관제까지 한 저장소에서 관리합니다.

- 대상 환경: Jetson Orin 계열, Ubuntu 22.04, ROS 2 Humble
- 센서: Livox MID-360 3D LiDAR + 내장 6축 IMU
- 차량: 2축 조향, 4륜 구동, 후륜 보조조향(RWS) 방식 4WIS
- 워크스페이스: `ALM_auto_ws/`
- 현행 측위: FPFH 대응점 → TEASER++ 전역 정합 → 지역 GICP → FAST-LIO 추적

## 시스템 구성

```text
Livox MID-360
  ├─ UDP 56301 ─ alm_sensors/livox_udp_pointcloud2
  │                ├─ /livox/lidar (PointCloud2 + per-point time)
  │                └─ 자기가림 마스크
  └─ UDP 56401 ─ alm_sensors/livox_udp_imu
                   └─ /livox/imu

[매핑]
  /livox/lidar + /livox/imu
    └─ FAST-LIO2 ─ /map_save ─ maps/<name>/cloud.pcd
                                      ├─ pcd2pgm ─ grid.pgm + grid.yaml
                                      └─ fpfh_map_builder ─ fpfh_map*

[측위]
  현재 스캔 + cloud.pcd + fpfh_map*
    └─ FPFH + TEASER++ + GICP ─ /icp_result ─ TF map → odom
    └─ FAST-LIO-Localization ─ /Odometry ─ TF odom → base_link

[자율주행]
  Nav2: SmacPlannerHybrid → ConstrainedSmoother → MPPI
    └─ /cmd_vel
       └─ cmd_arbiter ─ /cmd_vel_mux
          └─ command_manager ─ /mcu/command
             └─ mcu_bridge ─ UART v2 ─ STM32 ─ 2축 조향 + 4륜 구동
```

주행 시 TF는 `map → odom → base_link → steer/wheel` 순서입니다.
`map → odom`은 초기 정합 결과가, `odom → base_link`는 FAST-LIO-Localization이
담당합니다. 따라서 자율주행 통합 launch는 EKF를 끄고 TF 중복을 막습니다.

## 현재 기준값

| 항목 | 현재 값 | 기준 파일 |
|---|---:|---|
| 휠베이스 | 1.000 m | `alm_base_control/config/base_control.yaml` |
| 윤거 | 0.919 m | 같은 파일 |
| 후륜 보조조향률 | 0.5 | 같은 파일 |
| 일반 주행 최소 선회반경 | 1.643 m | 위 기구값에서 계산 |
| 전진/후진 속도 제한 | +0.20 / -0.07 m/s | `base_control.yaml`, `nav2.yaml` |
| 최대 회전속도 | 0.45 rad/s | 같은 파일 |
| Nav2 플래너 | SmacPlannerHybrid, Reeds-Shepp | `alm_navigation/config/nav2.yaml` |
| 경로 평활화 | ConstrainedSmoother | 같은 파일 |
| 경로 추종 | MPPI, DiffDrive 모델 | 같은 파일 |
| 활성 맵 | `cschool` | `alm_navigation/maps/active.yaml` |
| STM32 UART | `/dev/ttyTHS1`, 115200 8N1 | `alm_mcu_interface/config/mcu_interface.yaml` |

Nav2가 만든 경로는 최소 선회반경을 지키고, `command_manager`가 MCU 전송 직전에
조향 한계를 다시 적용합니다. 전역 경로가 표현하지 못하는 큰 시작 헤딩 오차는
제어단의 `ALIGN` 기동이 spin 모드로 보정합니다.

## 저장소 구조

| 경로 | 역할 |
|---|---|
| [`ALM_auto_ws/src/alm_bringup`](ALM_auto_ws/src/alm_bringup/) | 로봇, 매핑, 자율주행, WebUI 통합 launch |
| [`ALM_auto_ws/src/alm_sensors`](ALM_auto_ws/src/alm_sensors/) | MID-360 UDP 직접 파서, IMU relay, PointCloud→LaserScan |
| [`ALM_auto_ws/src/alm_navigation`](ALM_auto_ws/src/alm_navigation/) | FAST-LIO, 자동 초기측위, 맵 관리, Nav2, RViz |
| [`ALM_auto_ws/src/alm_base_control`](ALM_auto_ws/src/alm_base_control/) | 동작권 중재, 주행 모드, 4WIS 변환, 안전 게이팅 |
| [`ALM_auto_ws/src/alm_mcu_interface`](ALM_auto_ws/src/alm_mcu_interface/) | Jetson↔STM32 UART 브리지 |
| [`ALM_auto_ws/src/alm_description`](ALM_auto_ws/src/alm_description/) | 4WIS URDF/Xacro와 RViz 모델 |
| [`ALM_auto_ws/src/alm_msgs`](ALM_auto_ws/src/alm_msgs/) | MCU, 맵, WebUI, 음성용 사용자 메시지/서비스 |
| [`ALM_auto_ws/src/alm_web_backend`](ALM_auto_ws/src/alm_web_backend/) | 인증·제어권 잠금이 있는 WebUI 명령 백엔드 |
| [`alm-webui-v0.6`](alm-webui-v0.6/) | 브라우저 관제 화면 |
| [`ALM_auto_ws/src/thirdparty`](ALM_auto_ws/src/thirdparty/) | FAST-LIO2-Localization, TEASER++, livox_ros_driver2 빌드 의존성 |

## 시작 전 확인

새로 clone한 저장소에서 바로 자율주행 launch를 실행하면 안 됩니다.

1. `cloud.pcd`와 `fpfh_map*`는 대용량/재생성 자산이라 Git에서 제외됩니다.
   현재 저장소에는 `grid.pgm`, `grid.yaml`, `manifest.yaml`만 포함됩니다.
2. 활성 맵 `cschool`로 측위하려면 아래 파일을 현장에서 생성하거나 별도로 받아야 합니다.

   ```text
   maps/cschool/cloud.pcd
   maps/cschool/fpfh_map.meta
   maps/cschool/fpfh_map_points.pcd
   maps/cschool/fpfh_map_normals.pcd
   maps/cschool/fpfh_map_fpfh.pcd
   ```

3. LiDAR 장착 위치와 자기가림 마스크 기본값은 아직 실차 확정 대상입니다.
4. 휠 실효반지름, 기어비, 일반 조향 한계, 최대 RPM, 크랩/spin 부호는
   `base_control.yaml`의 `##CONFIRM##` 항목을 실차와 대조해야 합니다.
5. STM32→Jetson State 프레임은 STM32 펌웨어에 아직 구현되지 않았습니다.
   그동안 `/mcu/state`, `/wheel_odom`, `/joint_states`와 MCU fault 피드백은 나오지
   않으며, 자율주행 odometry는 FAST-LIO의 `/Odometry`를 사용합니다.

실차 투입 전에는 반드시 [`SETUP_CHECKLIST.md`](SETUP_CHECKLIST.md)를 확인하세요.

## 설치와 빌드

### 1. 시스템 의존성

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake libeigen3-dev \
  python3-colcon-common-extensions python3-rosdep \
  python3-serial python3-numpy python3-yaml python3-pil \
  ros-humble-robot-localization \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-smac-planner ros-humble-nav2-constrained-smoother \
  ros-humble-nav2-mppi-controller \
  ros-humble-pcl-ros pcl-tools \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools ros-humble-rviz2
```

WebUI를 사용할 때만 추가합니다.

```bash
sudo apt install -y ros-humble-foxglove-bridge nodejs npm
```

### 2. Livox-SDK2

런타임 센서 수신은 `alm_sensors`의 UDP 직접 파서가 담당합니다. 다만 FAST-LIO가
`livox_ros_driver2` 메시지 헤더에 빌드 의존하므로 Livox-SDK2와 메시지 패키지는
빌드 시 필요합니다.

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 3. 저장소 준비와 colcon build

```bash
cd ~
git clone https://github.com/Duhun12228/ALM_Autunomous.git
cd ALM_Autunomous/ALM_auto_ws
source /opt/ros/humble/setup.bash

# vendored livox_ros_driver2를 ROS 2 패키지로 인식시킨다.
cp src/thirdparty/livox_ros_driver2/package_ROS2.xml \
   src/thirdparty/livox_ros_driver2/package.xml

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys livox_ros_driver2

colcon build --symlink-install --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF
source install/setup.bash
```

새 터미널마다 다음 두 환경을 source합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ALM_Autunomous/ALM_auto_ws/install/setup.bash
```

처음 설치하는 Jetson의 전체 절차는 [`docs/JETSON_SETUP.md`](docs/JETSON_SETUP.md)를
참고하세요.

## 하드웨어 설정

### Livox MID-360

현재 기본 네트워크는 다음과 같습니다.

| 항목 | 값 |
|---|---|
| Jetson IP | `192.168.1.5/24` |
| MID-360 IP | `192.168.1.147` |
| Point host port | `56301` |
| IMU host port | `56401` |

```bash
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.147
```

네트워크를 바꾸면
[`MID360_config.json`](ALM_auto_ws/src/alm_sensors/config/MID360_config.json)과
`livox_udp_pointcloud2.py`의 `HOST_IP`, `POINT_PORT`를 함께 수정해야 합니다.

자기가림 마스크는 기본 활성화되어 있습니다. 제거되는 점을 확인할 때는:

```bash
ros2 launch alm_sensors lidar.launch.py \
  mask_debug:=/livox/lidar_masked
```

마스크 값을 바꾸면 기존 `cloud.pcd`와 FPFH DB도 같은 전처리 기준으로 다시
만들어야 합니다.

### STM32 UART

```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo usermod -aG dialout $USER
```

재로그인 후 `/dev/ttyTHS1` 권한을 확인합니다. UART는 `mcu_bridge` 하나만 열어야
합니다. 두 프로세스가 동시에 포트를 열면 프레임 바이트가 섞입니다.

## 맵 자산 규약

맵 하나는 `maps/<맵이름>/` 폴더 하나입니다.

```text
ALM_auto_ws/src/alm_navigation/maps/
├─ active.yaml
└─ <맵이름>/
   ├─ manifest.yaml
   ├─ cloud.pcd
   ├─ grid.pgm
   ├─ grid.yaml
   ├─ fpfh_map.meta
   ├─ fpfh_map_points.pcd
   ├─ fpfh_map_normals.pcd
   └─ fpfh_map_fpfh.pcd
```

- `manifest.yaml`이 있어야 하나의 맵으로 인식됩니다.
- `active.yaml`의 `active:` 값이 기본 맵을 결정합니다.
- `cloud.pcd`가 바뀌면 `grid.*`와 `fpfh_map*`를 다시 생성해야 합니다.
- `fpfh_map.meta`에는 부모 PCD 지문과 전처리 파라미터가 기록됩니다.

## 실행

아래 예시는 저장소가 `~/ALM_Autunomous`에 있다고 가정합니다.

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAPS=$WS/src/alm_navigation/maps
MAP_NAME=cschool
MAP=$MAPS/$MAP_NAME

cd $WS
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. 센서와 기본 하드웨어 스택

```bash
ros2 launch alm_bringup robot.launch.py
```

이 launch는 URDF, 센서, EKF, `cmd_arbiter`, `command_manager`, `mcu_bridge`를
함께 실행합니다. 다른 터미널에서 확인합니다.

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

STM32 State 업링크가 구현되기 전에는 `/mcu/state`와 `/wheel_odom`이 없는 것이
현재 예상 동작입니다.

### 2. 3D 매핑

터미널에서 직접 매핑할 때는 시작 전에
[`fastlio_mid360.yaml`](ALM_auto_ws/src/alm_navigation/config/fastlio_mid360.yaml)의
`map_file_path`를 대상 `maps/<맵이름>/cloud.pcd`로 바꿉니다. 이 값은 FAST-LIO
자체 파라미터라 launch 인자로 자동 치환되지 않으며, 잘못 두면 기존 맵을
덮어씁니다.

전체 하드웨어 통합 launch:

```bash
ros2 launch alm_bringup slam.launch.py rviz:=true
```

센서와 매핑 엔진만 따로 실행하려면 서로 다른 터미널에서:

```bash
ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation slam.launch.py rviz:=true
```

두 방식을 동시에 실행하지 마세요. 공간을 천천히 훑은 뒤 저장합니다.

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### 3. 2D 격자와 초기측위 DB 생성

```bash
ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAP/cloud.pcd \
  --out $MAP/grid \
  --resolution 0.05 \
  --z-min 0.3 \
  --z-max 0.8

ros2 run icp_relocalization fpfh_map_builder \
  --map $MAP/cloud.pcd \
  --output-prefix $MAP/fpfh_map
```

`pcd2pgm`의 z 범위는 실제 맵의 벽과 바닥 높이에 맞게 조정합니다. 그다음
`maps/active.yaml`의 `active:`를 `MAP_NAME`과 맞춥니다.

### 4. 자동 초기측위 확인

서로 다른 터미널에서 실행합니다.

```bash
ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation localization.launch.py
ros2 run alm_navigation map_publisher.py
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz
```

정합 전 `accum_frames` 기본 10프레임을 누적하므로 로봇을 정지 상태로 둡니다.
현재 `main`의 로컬라이저는 `/initialpose`를 사용하지 않습니다.

```bash
ros2 topic echo /icp_result --once
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /Odometry
```

`map → odom`이 나오고 `/Odometry`가 계속 발행되면 초기 정합과 추적이 시작된
상태입니다.

### 5. 자율주행

맵 자산이 모두 준비되어 있으면 한 명령으로 하드웨어, 측위, Nav2를 실행합니다.

```bash
ros2 launch alm_bringup navigation.launch.py rviz:=true
```

이 launch는 `robot.launch.py use_ekf:=false`, map server,
FAST-LIO-Localization, Nav2를 함께 실행합니다. `map → odom`이 생긴 뒤 RViz에서
**Nav2 Goal** 또는 waypoint를 지정합니다.

다른 맵을 명시하려면:

```bash
ros2 launch alm_bringup navigation.launch.py rviz:=true \
  map:=$MAP/grid.yaml \
  map_pcd:=$MAP/cloud.pcd \
  fpfh_db_prefix:=$MAP/fpfh_map
```

기본 주행 모드는 `auto`입니다. 명시적으로 다시 설정할 때는:

```bash
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

### 6. 키보드 텔레옵

기본 하드웨어 스택이 실행 중인 상태에서:

```bash
ros2 run alm_base_control keyboard_teleop.py
```

- `t`: 텔레옵 동작권 획득
- `w/s`, `a/d`: 전후진과 회전
- `j/l`: 크랩 좌우 이동
- `r`: 자율 동작권으로 반납
- `space`: E-STOP
- `c`: E-STOP 해제 요청

텔레옵도 `cmd_arbiter → command_manager → mcu_bridge` 경로를 사용하므로 자율주행과
같은 제한과 안전 게이팅을 거칩니다.

E-STOP은 래치입니다. 토픽에 `false`를 보내는 것으로 풀리지 않습니다.

```bash
ros2 service call /emergency_stop/release \
  alm_msgs/srv/ReleaseEstop "{reason: 'manual check complete'}"
```

## WebUI

WebUI 읽기는 `foxglove_bridge:8765`, 쓰기는 인증된
`alm_web_backend:8081`로 분리됩니다. 백엔드는 토큰이 없으면 기동을 거부합니다.

개발용 더미 MCU 스택:

```bash
export ALM_WEB_TOKEN=$(openssl rand -hex 16)
ros2 launch alm_bringup webui_dev.launch.py
```

프런트엔드:

```bash
cd ~/ALM_Autunomous/alm-webui-v0.6
npm ci
npm run build
npm run serve
```

브라우저에서 `http://<Jetson-IP>:8080`을 열고 설정 화면에 백엔드 주소
`http://<Jetson-IP>:8081`과 토큰을 입력합니다.

`webui_dev.launch.py`는 기본적으로 `fake_mcu`를 실행하고 `pcd_replay`는 끕니다.
실제 `robot.launch.py`가 이미 실행 중이면 중복 노드를 막기 위해 다음처럼 실행합니다.

```bash
ros2 launch alm_bringup webui_dev.launch.py \
  use_base_control:=false \
  use_fake_mcu:=false
```

저장된 PCD를 가짜 실시간 센서처럼 재생하는 기능은 명시할 때만 켜집니다.

```bash
ros2 launch alm_bringup webui_dev.launch.py use_pcd_replay:=true
```

화면에서 보이는 점군이 실제 센서인지 저장 맵 재생인지 혼동하지 마세요.

## 주요 ROS 인터페이스

| 이름 | 형식 | 역할 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | per-point time이 포함된 MID-360 점군 |
| `/livox/imu` | `sensor_msgs/Imu` | MID-360 내장 IMU |
| `/scan` | `sensor_msgs/LaserScan` | Nav2 costmap용 2D 스캔 |
| `/icp_result` | `geometry_msgs/PoseWithCovarianceStamped` | 확정된 전역 초기 자세 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 실시간 추적 |
| `/plan`, `/plan_smoothed` | `nav_msgs/Path` | 전역 원본/평활 경로 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 출력 |
| `/cmd_arbiter/set_owner` | `alm_msgs/srv/SetControlOwner` | 자율/텔레옵 동작권 전환 |
| `/mcu/command` | `alm_msgs/McuCommand` | STM32로 내려갈 최종 명령 |
| `/emergency_stop` | `std_msgs/Bool` | E-STOP 래치 설정 |
| `/emergency_stop/release` | `alm_msgs/srv/ReleaseEstop` | E-STOP 해제 요청 |
| `/map_save` | `std_srvs/srv/Trigger` | FAST-LIO 3D 맵 저장 |

## 검증 도구

기구값과 Nav2/URDF 설정 정합성:

```bash
ros2 run alm_navigation nav2_kinematic_check.py
```

ROS 없이 4WIS 변환과 ALIGN 자체시험:

```bash
python3 ALM_auto_ws/src/alm_base_control/scripts/fourwis_encode.py
python3 ALM_auto_ws/src/alm_base_control/scripts/path_align.py
```

측위 없이 실제 2D 맵에서 전역 계획과 평활화만 확인:

```bash
# 터미널 1
ros2 launch alm_navigation planner_check.launch.py x:=0.0 y:=0.0 yaw:=0.0

# 터미널 2
ros2 run alm_navigation plan_probe.py --goal -12.1 2.9 0
```

`planner_check`의 TF는 가짜이므로 실차 측위 스택과 동시에 실행하지 마세요.

## 알려진 제한과 안전 주의

- STM32 State 업링크는 예정 규격만 있고 펌웨어 송신부는 아직 없습니다.
- `McuCommand`의 E-STOP flag를 STM32 파서가 아직 사용하지 않습니다. 소프트웨어
  E-STOP만 믿지 말고 물리 비상정지 경로를 별도로 확보해야 합니다.
- `wheel_radius_m`, `gear_ratio`, `max_steer_deg`, `max_rpm`, 주행 방향 부호는
  실차 확인 전 잠정값입니다.
- LiDAR `base_link → livox_frame` 위치와 자기가림 마스크도 실측 확정 전입니다.
- Nav2 설정과 자체검증은 갖춰져 있지만 전체 실차 경로 추종은 환경별 재검증이
  필요합니다.
- `cloud.pcd`와 FPFH DB가 없는 새 clone은 측위·자율주행 준비가 끝난 상태가
  아닙니다.
- 현재 저장소에는 자동 CI가 없습니다. Jetson에서 colcon build와 검증 도구를
  실행한 결과를 기준으로 배포하세요.

## 문제 해결

| 증상 | 우선 확인 |
|---|---|
| `/livox/lidar`가 없음 | Jetson IP, MID-360 IP, UDP 56301, 방화벽 |
| `/livox/imu`가 없음 | UDP 56401, `MID360_config.json` |
| `/scan`이 비어 있음 | LiDAR 높이와 `pointcloud_to_scan` 높이 필터 |
| 측위 launch가 맵을 못 읽음 | 활성 맵의 `cloud.pcd`와 FPFH DB 존재 여부 |
| `[MATCH]`가 반복됨 | DB feature 수, PCD-DB 지문, 전처리 파라미터 |
| `Waiting for initial pose...` | `/icp_result`, TEASER/GICP 로그, 로봇 정지 여부 |
| Nav2가 odom을 못 찾음 | `/Odometry`, `map→odom→base_link` TF |
| `/mcu/command`는 있으나 움직이지 않음 | UART 권한, baud, STM32 mode/부호, 물리 E-STOP |
| `/mcu/state`가 없음 | 현재 STM32 업링크 미구현이면 예상 동작 |
| WebUI 백엔드가 기동하지 않음 | `ALM_WEB_TOKEN` 설정 여부 |
| 노드나 토픽이 두 개씩 보임 | 기존 launch와 `webui_dev` 중복 실행 여부 |

## 문서

- [Jetson 설치와 첫 실행](docs/JETSON_SETUP.md)
- [매핑부터 자율주행까지 운영 절차](docs/OPERATION_GUIDE.md)
- [자율주행 제어 파이프라인](docs/control_pipeline.md)
- [제어 피드백 현황과 한계](docs/control_feedback_analysis.md)
- [Nav2 계획과 기구 제약](docs/nav2_planning.md)
- [자율/텔레옵 동작권 중재](docs/control_arbitration.md)
- [Jetson↔STM32 UART 연동](docs/uart.md)
- [UART v2 프레임 규격](ALM_auto_ws/src/alm_mcu_interface/docs/uart_protocol.md)
- [현재 남은 작업](docs/TODO.md)
- [변경 이력](docs/CHANGES.md)
