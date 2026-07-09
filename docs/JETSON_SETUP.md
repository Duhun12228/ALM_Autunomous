# Jetson 설치 & 실행 가이드

새 Jetson에서 현재 워크스페이스를 받아 FAST-LIO 기반 자율주행까지 올리는 절차입니다.

- 대상: Jetson Orin 계열, Ubuntu 22.04, JetPack 6.x, ROS 2 Humble
- 센서: Livox MID-360, 이더넷 UDP 직접 수신
- 제어: STM32, UART `/dev/ttyTHS1`
- 현행 런타임 구조: livox_ros_driver2 노드 대신 `alm_sensors`의 UDP 직접 파서 사용

## 0. 사전 확인

```bash
lsb_release -a
uname -m
```

기대값:

- Ubuntu 22.04
- `aarch64`

## 1. ROS 2 Humble 설치

이미 설치되어 있으면 건너뜁니다.

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## 2. 의존 패키지 설치

```bash
sudo apt install -y \
  git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep \
  python3-serial python3-numpy python3-yaml python3-pil \
  ros-humble-robot-localization \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-pcl-ros pcl-tools \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools rviz2
```

rosdep 초기화:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

> 런타임에서는 livox_ros_driver2 노드를 실행하지 않습니다. 다만 vendored FAST-LIO/ICP 코드가
> `livox_ros_driver2` 메시지 헤더를 빌드 의존성으로 갖고 있어, 전체 colcon build에는
> Livox-SDK2가 필요할 수 있습니다.

## 3. Livox-SDK2 설치

현재 센서 수신은 UDP 직접 파서가 담당하지만, `src/thirdparty/livox_ros_driver2` 패키지를
전체 워크스페이스와 함께 빌드하려면 SDK 라이브러리가 필요합니다.

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 4. 워크스페이스 받기

```bash
cd ~
git clone <이_리포_URL> ALM_Autunomous
cd ~/ALM_Autunomous/ALM_auto_ws
source /opt/ros/humble/setup.bash
```

이미 받은 저장소라면:

```bash
cd ~/ALM_Autunomous
git pull
cd ALM_auto_ws
```

## 5. rosdep 및 빌드

`livox_ros_driver2`는 소스가 `src/thirdparty`에 vendoring 되어 있습니다.
rosdep 키가 환경에 따라 해결되지 않을 수 있으므로 skip합니다.

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys livox_ros_driver2
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
echo "source ~/ALM_Autunomous/ALM_auto_ws/install/setup.bash" >> ~/.bashrc
```

새 터미널 기본 source:

```bash
source /opt/ros/humble/setup.bash
source ~/ALM_Autunomous/ALM_auto_ws/install/setup.bash
```

## 6. 하드웨어 연결

### 6-1. STM32 UART

Jetson의 `/dev/ttyTHS1`을 시리얼 콘솔이 잡고 있으면 해제합니다.

```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo usermod -aG dialout $USER
```

그룹 권한은 재로그인 후 적용됩니다. 임시 확인용:

```bash
sudo chmod 666 /dev/ttyTHS1
```

설정 파일:

```text
ALM_auto_ws/src/alm_mcu_interface/config/mcu_interface.yaml
```

기본값:

- port: `/dev/ttyTHS1`
- baudrate: `115200`

### 6-2. Livox MID-360 네트워크

현재 설정:

- Jetson host IP: `192.168.1.5`
- LiDAR IP: `192.168.1.147`
- point host port: `56301`
- IMU host port: `56401`

Jetson 이더넷 IP 예시:

```bash
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.147
```

영구 설정은 NetworkManager 또는 netplan으로 구성합니다.

주의:

- `alm_sensors/config/MID360_config.json`의 host/LiDAR IP를 실제 장비에 맞춥니다.
- `livox_udp_pointcloud2.py`는 현재 `HOST_IP=192.168.1.5`, `POINT_PORT=56301`이
  상수입니다. 네트워크 변경 시 이 파일도 확인합니다.

## 7. 첫 실행 확인

```bash
cd ~/ALM_Autunomous/ALM_auto_ws
source install/setup.bash
ros2 launch alm_bringup robot.launch.py
```

다른 터미널:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /scan
ros2 topic hz /mcu/state
ros2 topic hz /wheel_odom
ros2 run tf2_tools view_frames
```

STM32가 아직 없으면 `/mcu/state`, `/wheel_odom`은 나오지 않는 것이 정상입니다.

## 8. 3D 매핑

```bash
WS=~/ALM_Autunomous/ALM_auto_ws
MAPS=$WS/src/alm_navigation/maps

ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation slam.launch.py rviz:=true
```

공간을 천천히 훑은 뒤:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

기본 PCD:

```text
$MAPS/alm_3d_map.pcd
```

## 9. 2D 맵 생성

```bash
ros2 run alm_navigation pcd2pgm.py \
  --pcd $MAPS/alm_3d_map.pcd \
  --out $MAPS/alm_map \
  --resolution 0.05 \
  --z-min 0.3 \
  --z-max 0.8
```

출력:

```text
$MAPS/alm_map.pgm
$MAPS/alm_map.yaml
```

## 10. 측위 검증

```bash
ros2 launch alm_sensors lidar.launch.py
ros2 launch alm_navigation localization.launch.py map_pcd:=$MAPS/alm_3d_map.pcd
```

RViz 확인:

```bash
ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=$MAPS/alm_map.yaml
rviz2 -d $WS/install/alm_navigation/share/alm_navigation/rviz/localization.rviz
```

필요하면 RViz의 **2D Pose Estimate**로 초기 위치를 지정합니다.

## 11. 자율주행

```bash
ros2 launch alm_bringup navigation.launch.py \
  map:=$MAPS/alm_map.yaml \
  map_pcd:=$MAPS/alm_3d_map.pcd
```

RViz에서 **2D Pose Estimate** 후 **Nav2 Goal**을 지정합니다.

주행 모드 확인:

```bash
ros2 topic echo /drive_mode/effective
```

auto 모드 설정:

```bash
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
```

## 12. 문제 해결

| 증상 | 확인 |
|---|---|
| `/livox/lidar` 없음 | Jetson IP, LiDAR IP, UDP port, 방화벽/인터페이스 |
| `/livox/imu` 없음 | IMU host port `56401`, LiDAR 설정 |
| `/scan` 비어 있음 | LiDAR 장착 높이, `pointcloud_to_scan` 높이 필터 |
| `FAST-LIO` 초기화 안 됨 | IMU 방향/가속도, per-point time 필드, 정지 초기화 |
| ICP 수렴 안 됨 | 초기 위치, `map_pcd`, voxel leaf size, prior map 품질 |
| Nav2 odom 에러 | `/Odometry` 토픽, TF `map->odom->base_link` |
| 로봇 안 움직임 | `/cmd_vel`, `/mcu/command`, `/mcu/state`, e-stop, fault |
| UART permission denied | dialout 그룹, 재로그인, `nvgetty` 해제 |

## 13. 현재 TODO와 연결되는 항목

- ICP `map_voxel_leaf_size` 0.5 -> 0.2 튜닝 검토
- Python UDP point parser의 CPU 부하 개선
- `map_pcd` launch 인자와 FAST-LIO `prior_map_path` 동기화
- 실차 본체 연결 상태에서 Nav2 goal 주행 검증
