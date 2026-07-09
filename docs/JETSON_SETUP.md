# Jetson 설치 & 실행 가이드 (처음부터)

새 Jetson에서 이 워크스페이스를 받아 자율주행까지 올리는 전체 단계입니다.
복사-붙여넣기로 진행할 수 있게 정리했습니다.

- 대상: **Jetson (Orin 등) + Ubuntu 22.04 (JetPack 6.x) + ROS 2 Humble**
- 센서: Livox MID-360 (3D LiDAR + 내장 6축 IMU), 이더넷 연결
- 제어: STM32 (UART `/dev/ttyTHS1`)

---

## 0. 사전 확인
```bash
lsb_release -a          # Ubuntu 22.04 여야 함 (Humble)
uname -m                # aarch64
```

## 1. ROS 2 Humble 설치 (이미 있으면 건너뛰기)
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release software-properties-common
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

## 2. 빌드 도구 & ROS 의존 패키지
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep git build-essential cmake
sudo rosdep init 2>/dev/null; rosdep update

# 우리 스택이 쓰는 ROS 패키지
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-tf2-tools \
  python3-serial
```

## 3. Livox-SDK2 설치 (드라이버 전제)
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc) && sudo make install
sudo ldconfig
```

## 4. livox_ros_driver2 빌드 (별도 워크스페이스)
```bash
mkdir -p ~/ws_livox/src && cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble          # 이 스크립트가 Humble용으로 빌드해줌
echo "source ~/ws_livox/install/setup.bash" >> ~/.bashrc
source ~/ws_livox/install/setup.bash
```

## 5. 이 워크스페이스 받아서 빌드
```bash
cd ~
git clone <이_리포_URL> ALM_Autunomous      # 이미 있으면 git pull
cd ~/ALM_Autunomous/ALM_auto_ws

# package.xml 의존성 자동 설치 (livox 는 위에서 소스빌드했으니 skip)
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys livox_ros_driver2

colcon build --cmake-args -DBUILD_TESTING=OFF
echo "source ~/ALM_Autunomous/ALM_auto_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

> 매번 새 터미널에서는: `source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash && source ~/ALM_Autunomous/ALM_auto_ws/install/setup.bash`
> (.bashrc 에 넣어놨으면 자동)

---

## 6. 하드웨어 연결 설정

### 6-1. STM32 UART 포트 (`/dev/ttyTHS1`)
Jetson 은 ttyTHS1 을 시리얼 콘솔(nvgetty)이 점유할 수 있으니 해제:
```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo usermod -aG dialout $USER      # 재로그인 후 적용
# 임시 권한:  sudo chmod 666 /dev/ttyTHS1
```

### 6-2. Livox MID-360 네트워크 (이더넷)
MID-360 은 고정 IP 라 Jetson 쪽도 같은 대역으로 맞춰야 합니다.
- Jetson 이더넷 IP 를 `192.168.1.5` 로 설정 (config 의 host ip 와 일치)
- LiDAR IP 확인 후 `src/alm_sensors/config/MID360_config.json` 의 `lidar_configs[0].ip` 수정
  (MID-360 기본 IP 는 `192.168.1.1XX` 형식, XX=시리얼 끝 2자리)
```bash
# 예: 이더넷 인터페이스가 eth0 인 경우
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.12       # LiDAR IP 로 응답 오면 OK
```
> 영구 설정은 netplan 또는 NetworkManager 로.

---

## 7. 실행

### 7-1. 센서/TF 먼저 확인 (하드웨어 붙인 직후)
```bash
ros2 launch alm_bringup robot.launch.py
# 다른 터미널
ros2 topic hz /scan               # 2D 스캔 나오나
ros2 topic hz /imu/data           # IMU 나오나
ros2 topic echo /odometry/filtered --once   # EKF odom (STM32 연결돼야 정상)
ros2 run tf2_tools view_frames    # map↔odom↔base_link↔livox_frame 확인
```

### 7-2. 맵 만들기 (조이스틱으로 방 전체 주행)
```bash
ros2 launch alm_bringup slam.launch.py
# 다른 터미널에서 RViz
rviz2 -d ~/ALM_Autunomous/ALM_auto_ws/src/alm_description/rviz/alm.rviz
# 방 다 돌았으면 저장
mkdir -p ~/ALM_Autunomous/maps
ros2 run nav2_map_server map_saver_cli -f ~/ALM_Autunomous/maps/my_map
```

### 7-3. 자율주행
```bash
ros2 launch alm_bringup navigation.launch.py map:=~/ALM_Autunomous/maps/my_map.yaml
# RViz: "2D Pose Estimate" 로 초기위치 → "Nav2 Goal" 로 목표 지정
# 기본 drive_mode=auto → 회전 필요구간 spin, 직진 normal 자동 전환
```

자세한 운영/튜닝은 `docs/OPERATION_GUIDE.md`, 확인값은 `SETUP_CHECKLIST.md` 참고.

---

## 8. 자주 겪는 문제
| 증상 | 확인 |
|---|---|
| `/scan` 안 나옴 | LiDAR ping 되나, MID360_config.json IP, `lidar.launch` 의 min/max_height |
| `livox_ros_driver2` 못 찾음 | `source ~/ws_livox/install/setup.bash` 했나 |
| `/dev/ttyTHS1` permission denied | dialout 그룹/재로그인, nvgetty 해제, chmod 666 |
| odom 이상/AMCL 흔들림 | STM32 `/wheel_odom` 들어오나(`ros2 topic hz /wheel_odom`) |
| TF map→odom 없음 | 매핑은 slam.launch, 자율주행은 navigation.launch (둘 중 하나만) |
| 로봇 안 움직임 | `/cmd_vel`→`/mcu/command` 나오나, `enable_motors:true`인가, e-stop/estop 상태 |
