# ALM 자율주행 운영 가이드

현재 워크스페이스 기준 운영 순서입니다. 이 프로젝트는 기존 2D
`slam_toolbox + AMCL + EKF` 흐름에서 벗어나, **FAST-LIO2 3D 매핑**과
**FAST-LIO-Localization 측위**를 사용합니다.

핵심 실행 경로:

```text
Livox MID-360 UDP 직접 파싱
  -> /livox/lidar, /livox/imu, /scan
FAST-LIO2 매핑
  -> alm_3d_map.pcd
pcd2pgm
  -> alm_map.pgm/yaml
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
~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/alm_3d_map.pcd
```

PCD는 대용량 로컬 산출물이므로 `.gitignore`에서 제외됩니다.

## 3. 3D PCD를 Nav2용 2D 맵으로 변환

Nav2의 global costmap static layer에는 2D occupancy map이 필요합니다.
FAST-LIO가 만든 PCD를 `pcd2pgm.py`로 변환합니다.

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAPS=$WS/src/alm_navigation/maps

ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAPS/alm_3d_map.pcd \
  --out $MAPS/alm_map \
  --resolution 0.05 \
  --z-min 0.3 \
  --z-max 0.8
```

출력:

```text
alm_map.pgm
alm_map.yaml
```

`pcd2pgm.py`가 출력하는 z 분포를 보고 벽/장애물만 잡히도록
`--z-min`, `--z-max`를 조정합니다.

## 4. 측위만 검증

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAPS=$WS/src/alm_navigation/maps

ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation localization.launch.py map_pcd:=$MAPS/alm_3d_map.pcd
```

2D 맵을 RViz에 함께 띄워 확인:

```bash
ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=$MAPS/alm_map.yaml
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz
```

시작 위치가 맵 원점과 다르면 RViz의 **2D Pose Estimate**로 실제 위치와 방향을
한 번 지정합니다. ICP가 수렴하면 `map->odom`이 잡히고 FAST-LIO가
`odom->base_link`를 계속 추적합니다.

## 5. 자율주행

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAPS=$WS/src/alm_navigation/maps

ros2 launch alm_bringup navigation.launch.py \
  map:=$MAPS/alm_map.yaml \
  map_pcd:=$MAPS/alm_3d_map.pcd
```

이 launch는 다음을 함께 올립니다.

- `robot.launch.py use_ekf:=false`
- `nav2_map_server`
- FAST-LIO-Localization
- Nav2 planner/controller/BT

RViz에서 **2D Pose Estimate**로 초기 위치를 주고 **Nav2 Goal**을 지정합니다.
Nav2는 `/cmd_vel`을 만들고, `command_manager`가 `auto` 모드에서
`normal/spin/crab` 중 실제 MCU에 보낼 모드를 선택합니다. 현재 crab은 기본 비활성입니다.

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

## 8. 현재 운영상 주의

- `localization.launch.py`의 `map_pcd`와
  `fastlio_relocalization.yaml`의 `prior_map_path`가 같은 PCD를 가리켜야 합니다.
- 런타임에서는 livox_ros_driver2 노드를 실행하지 않고 UDP 직접 파서를 사용합니다.
- `livox_udp_pointcloud2.py`의 host IP/point port는 현재 상수입니다.
  네트워크를 바꾸면 스크립트 값도 확인해야 합니다.
- Python UDP 파서는 부하가 큽니다. TODO에 적힌 대로 C++ 이식 또는 필터링 튜닝이
  장시간 실주행 전 우선 과제입니다.
