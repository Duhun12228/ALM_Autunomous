# ALM 자율주행 운영 가이드 (맵 작성 → 저장 → 자율주행)

하드웨어 완성 + LiDAR 위치 확정(URDF 완성) 이후, 실제로 방을 매핑하고 자율주행하는
전체 순서입니다. 아래 내용은 현재 워크스페이스 설정을 그대로 확인한 것입니다.

---

## 0. 센서 사용 구조 — 2D는 SLAM, 3D는 회피

자주 헷갈리는 부분이라 먼저 정리:

| 용도 | 사용 센서 | 근거 (설정 파일) |
|---|---|---|
| **SLAM 맵 작성** | **2D `/scan`만** | `slam_toolbox.yaml`: `scan_topic: /scan` |
| **AMCL 위치추정** | **2D `/scan`만** | `nav2.yaml` amcl: `scan_topic: /scan` |
| **Nav2 장애물 회피(costmap)** | **2D `/scan` + 3D `/livox/lidar` 둘 다** | `nav2.yaml`: `observation_sources: scan pointcloud` |

- `/scan` 은 Livox 3D 포인트클라우드를 `pointcloud_to_scan` 이 지면 근처 한 층만 잘라 만든 2D 라인입니다. → 지도/측위는 가볍고 안정적인 2D로.
- 회피는 2D가 놓치는 낮은/높은 장애물(선반 다리, 튀어나온 물체)까지 잡으려고 **3D 포인트클라우드를 costmap에 추가**로 넣었습니다. `min_obstacle_height: 0.10 ~ max: 1.5 m` 범위의 점만 장애물로.
- 즉 **맵은 2D, 실시간 회피는 2D+3D**. 원하던 구조 맞습니다.

> map→odom TF 는 매핑 때는 slam_toolbox 가, 자율주행 때는 AMCL 이 담당합니다.
> 둘은 서로 다른 launch(`slam.launch.py` vs `navigation.launch.py`)라 **동시에 켜지 않습니다.**

---

## 1. 매핑 (조이스틱으로 방 전체 돌기)

```bash
cd ~/ALM_Autunomous/ALM_auto_ws && source install/setup.bash

# 상시 스택(센서+EKF+제어+MCU) + slam_toolbox 를 한 번에
ros2 launch alm_bringup slam.launch.py
```

그러면 뜨는 것: Livox 드라이버 → `/scan`, EKF → `odom→base_link` TF, slam_toolbox → `/map` + `map→odom` TF.

**RViz로 맵 그려지는 것 보기** (다른 터미널):
```bash
rviz2 -d ~/ALM_Autunomous/ALM_auto_ws/src/alm_description/rviz/alm.rviz
```

**조이스틱/키보드로 방 전체 주행**하며 맵을 채웁니다. (드라이브 모드는 기본 `auto`)
- 천천히, 벽을 다 훑도록. 같은 곳으로 되돌아오면(loop closure) 맵이 더 정확해집니다.

---

## 2. 맵 저장 — 어디에, 어떻게

맵이 충분히 그려졌으면 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ALM_Autunomous/maps/my_map
```
→ `~/ALM_Autunomous/maps/my_map.pgm` + `my_map.yaml` 두 파일 생성.

### ⚠️ 저장 위치 관련 주의 (중요)
- **권장: 워크스페이스 밖 고정 경로**(`~/ALM_Autunomous/maps/`)에 저장하고, 자율주행 시 그 절대경로를 `map:=` 로 넘기세요. → 재빌드 불필요, 제일 깔끔.
- (대안) `src/alm_navigation/maps/` 안에 저장하면 **`colcon build` 를 다시 해야** `install/` 로 복사되어 기본 경로로 잡힙니다. 안 하면 navigation.launch 기본값이 옛 맵을 봅니다.

처음 한 번 폴더 생성:
```bash
mkdir -p ~/ALM_Autunomous/maps
```

---

## 3. 자율주행 (AMCL 초기화 → 목표 → Nav2)

```bash
cd ~/ALM_Autunomous/ALM_auto_ws && source install/setup.bash

ros2 launch alm_bringup navigation.launch.py \
  map:=~/ALM_Autunomous/maps/my_map.yaml
```
이거 하나로 상시 스택(센서+EKF+제어+MCU) + map_server + **AMCL** + Nav2(planner/controller) 가 전부 뜹니다.

### 3-1. 초기 위치 잡기 (필수 — AMCL은 시작 위치를 모름)
6축 IMU로는 지도상 절대 위치를 못 구하므로, **처음 한 번은 지금 로봇이 지도 어디에 있는지 알려줘야** 합니다.
- RViz 상단 **"2D Pose Estimate"** 클릭 → 지도에서 로봇 실제 위치에 화살표로 찍기(방향까지).
- 또는 명령으로:
  ```bash
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
  ```
- 초기 위치를 준 뒤 로봇을 조금 움직이면 파티클이 수렴하며 위치가 안정됩니다.

> 나중에 **항상 같은 자리(도킹 스테이션)에서 출발**하도록 정해지면, 이 초기위치를 자동 발행하는
> 헬퍼 노드를 붙여 RViz 클릭을 없앨 수 있습니다. (현재 보류)

### 3-2. 목표 찍고 자율주행
- RViz 상단 **"Nav2 Goal"** 클릭 → 지도에서 가고 싶은 지점 찍기.
- 그러면: planner(A*)가 `/plan` 생성 → controller(DWB)가 `/cmd_vel` 생성 → `command_manager`가 모드 해석(auto→normal/spin) + 안전 제한 → `/mcu/command` → **STM32가 바퀴 구동**.
- 기본 `drive_mode=auto` 라 별도 설정 없이 바로 됩니다. (회전 필요하면 자동 spin, 직진은 normal)

**여러 지점 순찰**(웨이포인트)은 RViz의 "Waypoint / Nav Through Poses" 모드로 여러 점을 큐잉하거나,
좌표 리스트를 자동 실행하는 미션 노드를 붙이면 됩니다. (미션 노드 현재 보류)

---

## 4. 한눈에 보는 데이터 흐름

```
[매핑]   Livox → /scan ─┐
                        ├─ slam_toolbox → /map, map→odom  →  map_saver → my_map.{pgm,yaml}
        EKF → odom→base_link ┘

[자율]   Livox → /scan ──→ AMCL(+map_server) → map→odom
         Livox → /scan + /livox/lidar ──→ Nav2 costmap(회피)
         Nav2(planner+controller) → /cmd_vel → command_manager → /mcu/command → STM32
         STM32 엔코더 → /wheel_odom ─┐
         Livox IMU → /imu/data ──────┴→ EKF → odom→base_link
```

---

## 5. 첫 브링업 점검 체크 (하드웨어 붙인 직후)
```bash
ros2 topic hz /scan            # 라이다 2D 나오나
ros2 topic hz /imu/data        # IMU 나오나
ros2 topic echo /odometry/filtered --once   # EKF 오도메트리 나오나
ros2 run tf2_tools view_frames # map↔odom↔base_link↔livox_frame 연결 확인
```
`/scan` 이 비어있으면 `pointcloud_to_scan` 의 `min_height/max_height`(라이다 마운트 높이 기준)를 조정하세요.
자세한 확인·수정 값은 [SETUP_CHECKLIST.md](../SETUP_CHECKLIST.md).
