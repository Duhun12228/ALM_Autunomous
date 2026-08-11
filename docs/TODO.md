# TODO — 남은 작업

3D LIO 측위 전환(FAST-LIO2 매핑 + FAST-LIO-Localization) 이후 남은 작업 목록.
최종 업데이트: 2026-08-10 (dev/fastlio2-sc: WebUI 명령 경로 Phase 3 — `alm_web_backend` + E-STOP 래치).

## ✅ 완료 (방식 A, 집에서 LiDAR 핸드헬드 검증)
- 센서 UDP 직접 파싱 (per-point time 포함), 런치 통합
- FAST-LIO2 매핑 → `maps/<맵이름>/cloud.pcd` (실주행맵 1,623,894점)
- pcd2pgm → `maps/<맵이름>/grid.pgm+yaml` (1014×671 @0.05m), z밴드 [0.3,0.8]
- fpfh_map_builder → `maps/<맵이름>/fpfh_map*` (1,414 features)
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
- [x] Python UDP 파서 벡터화 완료 (`np.frombuffer` + 구조화배열 `create_cloud`).
      합성부하 1초분(20만점) 388ms → 87ms (**4.5배**). 발행 바이트는 기존 구현과
      비트 단위 동일 — per-point `time` 보존 확인. 실센서 9.9Hz/181k pts/s 정상.
- [ ] 실차에서 Recv-Q 밀림이 실제로 해소됐는지 재확인. 남으면 C++ 이식 검토.

### 3.5 자기가림 마스크 (LiDAR 뒤 적재물)
- [x] 파서에 마스크 구현 (`mask_*` 파라미터, 방위 center±width + 수평거리 + z밴드).
      제거된 점은 `mask_debug` 토픽으로 확인 가능. 오프라인/실센서 검증 통과.
- [ ] **실측 튜닝**: 적재물 올린 상태로 `mask_debug` 보며 center/width/max_range 확정
      → `lidar.launch.py` 기본값에 반영 (현재값 180deg/60deg/1.5m 는 추정치).
- [ ] **마스크 ON 상태로 재매핑 → pcd2pgm → fpfh_map_builder 재실행**. 적재물이 0.5m
      밖까지 뻗어 있어 `blind: 0.5` 로 안 걸러졌으므로, 기존 `alm_lab/cloud.pcd` 에는
      적재물이 궤적을 따라 번져 기록돼 있다. 스캔에서만 빼면 domain gap 이 반대로 벌어진다.
- [ ] 재매핑 후 초기정합 성공률 재측정 — 적재물이 TEASER/ICP 실패에 기여했는지 확인
      (실험 체크리스트의 단독 변수 하나로 다룰 것).

### 3.9 WebUI 명령 연동 (Phase 3 완료 · 2026-08-10)
- [x] `alm_web_backend` 신설 — 브라우저 → 로봇 **쓰기** 경로. HTTP :8081, 표준 라이브러리만.
      읽기(`foxglove_bridge` :8765)의 차단 설정은 그대로 유지 — 브라우저가 ROS 토픽에
      직접 publish 하는 경로는 여전히 0개다.
- [x] **E-STOP 래치**를 `command_manager` 에 구현(`estop_latch: true`).
      `/emergency_stop` 의 `false` 는 무시하고, 해제는 `/emergency_stop/release`
      (`alm_msgs/srv/ReleaseEstop`) 로만. MCU 가 fault/estop 보고 중이면 거부.
- [x] 웹 세션 제어권(리스 15초) — `cmd_arbiter` 의 동작권과 **다른 축**임에 주의.
- [x] 실동작: SLAM launch 기동·종료, `/map_save`, `pcd2pgm`, `fpfh_map_builder`,
      맵 폴더 생성, 활성 맵 전환, 속도 한계 조회(UI 하드코딩 제거).
- [ ] **`/map_save` 만 실센서 검증 남음.** 지금 스택은 IMU 가 없어 FAST-LIO 가 점군을
      못 쌓는다. ⚠ 빈 상태로 저장하면 FAST-LIO 가 `pcl::IOException` 을 안 잡고 **죽는다**
      (상류 버그). backend 가 이 상황을 감지해 즉시 사유를 반환하도록 해뒀다.
- [ ] Phase 4 (측위·Nav2 명령), Phase 5 (수동주행) — 아래 조건 참고.
- [ ] ⚠ **Phase 5 선행조건**: `base_control.yaml` 의 4WIS 상수 8개(`##CONFIRM##`)를
      실차로 확정. 미확정 상태에서 웹 twist 를 보내면 그대로 엉뚱한 rpm/조향각이 된다.
      웹 텔레옵과 `keyboard_teleop` 의 `/cmd_vel_teleop` 경쟁 해소 방식도 먼저 결정할 것.

### 4. TF/구조 정리
- [ ] 매핑 모드에서 EKF 필요성 재검토 (맵에 무관 — 켤 이유 없으면 정리).
- [ ] fastlio가 odom→**sensor** 발행 (base_link 아님). 필요시 base_frame 정합/extrinsic 정리.
- [ ] degeneracy(빈 복도) 대비 엔코더(wheel_odom) 융합 여부 결정 — 세 브랜치 공통 하부구조.

### 5. 브랜치별 개발 (측위 3방식)
- [x] **`dev/fastlio2-sc`**: 자동초기화를 **FPFH+TEASER++** 로 확정.
      `fpfh_map_builder`(맵 feature DB) + `teaser_fpfh_localizer`(전역 대응점 →
      TEASER++ → 지역 GICP), `icp_node` 는 coarse/medium/fine 다단계.
      **실센서/실차 검증 남음.**
- [x] ~~Scan Context 재측위~~ **제거**. `scan_context.py`/`sc_build_db.py`/
      `sc_localizer.py` 는 FPFH+TEASER++ 로 교체되면서 런치에서 빠진 뒤 방치돼 있었고,
      이 브랜치에서 삭제했다. 코드는 `dev/sc-lio-sam` 과 이 브랜치 히스토리(`09e9dc3`)에
      남아 있다. `experiment/alm_experiment_gui.py` 의 SC 탭은 이제 동작하지 않는다.
      **WebUI 반영 완료(2026-08-07)**: 방식 A/B/C 선택 UI 제거, SC 문구 9곳 교체,
      `sc_db_035.npz` 삭제, `/sc_candidates` 를 foxglove allowlist 에서 제거
      (발행자가 없어졌다).
      남길 만한 노하우: 실내는 z밴드가 천장을 포함하면 안 됨(디스크립터 균일화),
      가상 키프레임 DB 는 오클루전을 넣어야 실제 스캔과 채움률이 맞음(12.1% vs 12.2%).
- [ ] **`dev/sc-lio-sam`**: SC-LIO-SAM(ROS2) 매핑 교체 + 루프클로저. GTSAM 빌드(ARM),
      6축 IMU 대응 필요. 공간 넓을 때만 가치.
- [ ] 세 방식 실차 비교(초기화 성공률·정확도·Orin Nano 부하).

## ⚠️ 알아둘 것
- 맵은 **폴더 하나 = 맵 하나**(`maps/<이름>/{manifest.yaml, cloud.pcd, grid.*, fpfh_map*}`).
  대용량 자산(`*.pcd`, `fpfh_map.meta`)은 `.gitignore` — 각 환경에서 매핑으로 생성.
  활성 맵은 `maps/active.yaml`. 자산 짝맞음은 `map_manager` 가 `/alm/map_inventory` 로 알린다.
- `icp_node`는 일회성(성공 시 자동종료 → `/prior_map` 사라짐 = 정상).
- **E-STOP 은 래치다.** `ros2 topic pub /emergency_stop ... "{data: false}"` 로는 안 풀린다.
  `ros2 service call /emergency_stop/release alm_msgs/srv/ReleaseEstop "{reason: '...'}"`.
- **`alm_web_backend` 는 `ALM_WEB_TOKEN` 없이는 안 뜬다** (fail-closed).
  `export ALM_WEB_TOKEN=$(openssl rand -hex 16)` 후 `webui_dev.launch.py`.
- `colcon --symlink-install` 함정 둘: ① `install(PROGRAMS)` 가 심링크라 **소스에 +x 가
  없으면 노드가 안 뜬다**(PermissionError). ② `install/.../config/*.yaml` 은 심링크라
  `os.replace` 로 쓰면 **심링크만 갈아치우고 소스는 그대로**다 — realpath 로 먼저 풀 것.
- 상세 운용 함정은 커밋 메시지/작업 이력 참고.
