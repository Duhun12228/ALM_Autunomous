# ALM Scan Context–ICP 실험 GUI

> ⚠ **`dev/fastlio2-sc` 에서는 SC 기능이 동작하지 않습니다.** 이 브랜치는 자동초기화를
> FPFH+TEASER++ 로 교체하면서 `sc_build_db.py`/`sc_localizer.py`/`scan_context.py` 를
> 삭제했습니다. 아래 SC 관련 탭은 `ros2 run` 단계에서 실패합니다.
> SC 실험은 `dev/sc-lio-sam` 브랜치에서 하세요. ICP 관련 탭은 그대로 쓸 수 있습니다.

## 목적

`dev/fastlio2-sc`, `dev/sc-lio-sam` 브랜치의 Scan Context DB 생성과 초기 위치 정합 실험을 터미널 명령 반복 없이 수행하기 위한 Python GUI입니다.

GUI는 다음 명령을 내부에서 직접 실행합니다.

- `ros2 run alm_navigation sc_build_db.py ...`
- `ros2 run alm_navigation sc_localizer.py --ros-args ...`
- `ros2 run icp_relocalization icp_node --ros-args ...`
- `ros2 run icp_relocalization transform_publisher ...`
- `ros2 run fast_lio fastlio_mapping --params-file ...`
- `ros2 launch alm_sensors lidar.launch.py`

## 현재 PC에서 화면만 보기

ROS가 설치되지 않은 PC에서는 자동으로 **미리보기 모드**가 활성화됩니다.

```bash
python3 alm_experiment_gui.py
```

미리보기 모드에서는 버튼을 눌러도 실제 ROS 명령은 실행하지 않고 하단 로그에 생성된 명령만 표시합니다.

`tkinter`가 없다면:

```bash
sudo apt install python3-tk
```

## Jetson 설치

```bash
sudo apt update
sudo apt install python3-tk python3-yaml
```

ROS 2 Humble과 워크스페이스가 빌드된 상태여야 합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ALM_Autunomous/ALM_auto_ws
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

GUI 실행:

```bash
python3 ~/ALM_Autunomous/alm_experiment_gui.py
```

## 주요 탭

1. **공통 설정**
   - 워크스페이스, PCD, SC DB, FAST-LIO YAML 경로
   - 센서 및 RViz 실행
   - 전체 프로세스 종료

2. **Baseline**
   - 기존 파라미터로 전체 위치 정합 실행

3. **SC 높이**
   - Z1/Z2/Z3 높이 범위로 DB 생성 및 비교

4. **DB Step**
   - 0.75/0.50/0.35 m 간격 DB 생성 및 비교

5. **ICP 비교**
   - Single ICP 직접 실행
   - SC 후보 캡처 → Coarse ICP → Fine ICP 자동 순차 실행

6. **누적 프레임**
   - `sc_localizer`의 누적 프레임 변경

7. **Timeout·후보**
   - 성공 횟수, 후보 timeout, topk, 후보 수 변경

8. **최종 통합**
   - 결과 CSV, GUI 로그, 파라미터 JSON 저장

## Coarse-to-Fine 작동 방식

현재 C++ 소스를 수정하지 않고 다음 순서로 두 개의 `icp_node`를 순차 실행합니다.

```text
sc_localizer 실행
→ /initialpose 1개 캡처
→ sc_localizer 중지
→ Coarse icp_node 실행
→ /icp_result 캡처
→ Coarse 노드 중지
→ Coarse 결과를 initial pose로 Fine icp_node 실행
→ 최종 /icp_result 캡처
```

이 기능은 센서 토픽이 발행 중이어야 합니다.

## 제한 사항

- 현재 저장소의 ICP 노드는 `/livox/lidar` 단일 프레임을 source로 사용합니다.
- GUI의 `누적 프레임` 탭은 현재 `sc_localizer`의 Scan Context 누적 프레임을 변경합니다.
- ICP 자체가 동일 누적 점군을 source로 사용하게 하려면 C++ 또는 토픽 입력 구조의 추가 수정이 필요합니다.
- 실제 정답 위치 센서가 없으므로 `정확함 / 약간 벗어남 / 오정합 / 실패`는 RViz 확인 후 사용자가 선택합니다.
- `fitness score`는 실행 로그에서 출력되는 형식에 따라 자동 파싱을 추가할 수 있으나, 현재 버전은 기본적으로 수동 기록과 전체 로그 저장을 제공합니다.

## 안전 기능

- 프로세스를 process group 단위로 종료
- GUI 종료 시 ROS 프로세스 일괄 종료
- ROS 미설치 환경에서 자동 미리보기
- 원본 launch 파일을 수정하지 않고 노드를 직접 실행
- 실험 결과와 파라미터를 별도 폴더에 저장
