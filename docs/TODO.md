# TODO — 남은 작업

3D LIO 측위 전환(FAST-LIO2 매핑 + FAST-LIO-Localization) 이후 남은 작업 목록.
최종 업데이트: 2026-07-10.

## ✅ 완료 (방식 A, 집에서 LiDAR 핸드헬드 검증)
- 센서 UDP 직접 파싱 (per-point time 포함), 런치 통합
- FAST-LIO2 매핑 → `alm_3d_map.pcd`(764k점, 8×7×3m, 드리프트 낮음)
- pcd2pgm → 2D 맵(`alm_map.pgm/yaml`), z밴드 [0.3,0.8]
- FAST-LIO-Localization 측위 성공 (자동수렴 + RViz 2D Pose Estimate 재측위)
- `map_publisher.py` + `localization.rviz`(2D 실시간 트래킹 뷰)
- Nav2 설치(apt)

## 🔜 다음 (우선순위 순)

### 1. 측위 튜닝
- [ ] `localization.launch.py` 의 `map_voxel_leaf_size` **0.5 → 0.2** (현재 76만점→517점 과다축소).
      cloud_voxel도 0.3→0.2 검토. 정합 정확도/안정성 개선.
- [ ] `fitness_score_thre`(0.2)·`converged_count_thre`(40) 실측 튜닝.
- [ ] pcd2pgm 생성 yaml 의 `free_thresh`(0.25) → 0.196 검토 (205=unknown 오독 방지).

### 2. Nav2 실주행 검증
- [ ] `alm_bringup navigation.launch.py` 로 측위+Nav2 통합 구동 (map/map_pcd 인자).
- [ ] costmap 관측소스(/scan + /livox/lidar) 실동작, planner/controller 파라미터 실차 튜닝.
- [ ] Nav2 Goal → /cmd_vel → command_manager(auto) → 실제 주행 확인.
- [ ] 로봇 본체(바퀴/MCU) 연결 상태에서 재검증 (지금까진 LiDAR 핸드헬드).

### 3. 파서 성능
- [ ] Python UDP 파서 CPU 84% (초당20만점 벅참, Recv-Q 밀림). **C++ 이식** 또는
      `point_filter_num`↑ 로 여유 확보.

### 4. TF/구조 정리
- [ ] 매핑 모드에서 EKF 필요성 재검토 (맵에 무관 — 켤 이유 없으면 정리).
- [ ] fastlio가 odom→**sensor** 발행 (base_link 아님). 필요시 base_frame 정합/extrinsic 정리.
- [ ] degeneracy(빈 복도) 대비 엔코더(wheel_odom) 융합 여부 결정 — 세 브랜치 공통 하부구조.

### 5. 브랜치별 개발 (측위 3방식)
- [ ] **`dev/fastlio2-sc`**: Scan Context 재측위 통합 → 초기위치 자동화(2D Pose Estimate 불필요).
      PolarisXQ엔 SC 없음(SAC-IA만) → scancontext 이식. MID-360 비반복스캔 대응(프레임 누적).
- [ ] **`dev/sc-lio-sam`**: SC-LIO-SAM(ROS2) 매핑 교체 + 루프클로저. GTSAM 빌드(ARM),
      6축 IMU 대응 필요. 공간 넓을 때만 가치.
- [ ] 세 방식 실차 비교(초기화 성공률·정확도·Orin Nano 부하).

## ⚠️ 알아둘 것
- `alm_3d_map.pcd`(대용량)는 `.gitignore` — 각 환경에서 매핑으로 생성.
- `icp_node`는 일회성(성공 시 자동종료 → `/prior_map` 사라짐 = 정상).
- 상세 운용 함정은 커밋 메시지/작업 이력 참고.
