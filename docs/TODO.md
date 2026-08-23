# TODO — 남은 작업

3D LIO 측위 전환(FAST-LIO2 매핑 + FAST-LIO-Localization) 이후 남은 작업 목록.
최종 업데이트: 2026-08-20 (feat/nav2-hybrid-astar: ALIGN 경로 헤딩 정렬 기동 + dwell 재유도).

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
- [x] **Phase 4-1 측위 명령** — 자동정합 기동/중단, 진행 로그, FPFH 파라미터 (2026-08-10).
- [x] ⭐ **Phase 4-2 자율주행 명령 연동** (2026-08-24). 웹에서 목표를 보내 실제로 주행한다.
      · 백엔드: `navigation` 프로세스 슬롯(+ slam/localization 과 3자 배타) ·
        `NavigateToPose`/`FollowWaypoints` 액션 클라이언트 · 미션 상태기계 ·
        `/api/navigation/{start,stop,goal,pause,resume,cancel,log}` 8개
      · 전송 전 점검(`navigation.py`): **Nav2 액션 광고 + map→odom TF** 를 본다.
        TF 를 보는 이유 — `/icp_result` 는 성공 시 한 번만 오므로 백엔드가 그 뒤에
        재시작하면 영영 못 본다. '지금 조회되는가' 는 재시작과 무관하게 참이다.
        스탬프 나이(5 s)까지 봐서 측위가 죽은 뒤의 잔상도 거른다.
      · 프런트: `주행 시작` 이 상태에 따라 **스택 기동 → 목표 전송** 두 단계를 한다.
        진척률은 Nav2 피드백에서만 만든다(단일 목표=남은거리, 웨이포인트=도달수/전체).
        FollowWaypoints 가 안 주는 남은거리는 **지어내지 않고 `—`** 로 둔다 —
        목업이 `setInterval` 로 progress 를 올려 '도착 안 했는데 미션 완료' 를
        띄우던 것이 이 화면의 원래 문제였다.
      · 반복/순환 주행은 목록을 그만큼 펴서 한 미션으로 보낸다(서버 반복호출 아님).
        '몇 바퀴째' 라는 상태가 서버·화면 양쪽에 생기는 것을 피하려는 것 — 목록
        하나면 `index` 하나로 전부 표현된다.
      · 검증(하드웨어 0): `alm_bringup/scripts/fake_nav2.py` 가 액션 서버 둘과
        map→odom TF 를 대역한다. `alm_web_backend/tools/nav_api_test.py` 22항목
        (수락·피드백·일시정지/재개·중단·abort·missed waypoints·좌표검증) 전부 통과,
        `alm-webui-v0.6/tools/nav-ui-test.mjs` 20항목(버튼 상태사상·진척률) 전부 통과.
      · ⚠ **실차 미검증.** 위는 전부 대역이다 — 실제 Nav2 · 실제 측위 · 실제 주행은
        아직 한 번도 안 돌렸다. 특히 **초기 정합이 아직 성공한 적이 없으므로**
        (README 79회 시도 0회) `tf_ready` 가 참이 되는 순간을 실차에서 본 적이 없다.
- [ ] ⚠ **자율주행 스택이 측위를 포함한다** — `navigation.launch.py` 안에
      `localization.launch.py` 가 들어 있어 '측위로 수렴을 확인한 뒤 Nav2 만 얹기' 가
      안 된다. 자율주행을 시작하면 초기 정합을 처음부터 다시 한다. 정합이 비싼
      이 브랜치에서 실제로 불편한 지점이고, 고치려면 Nav2 코어만 띄우는 launch 를
      따로 만들어야 한다. 배타 규칙이 하나 더 늘어나는 대가와 저울질할 것.
- [x] ⭐ **웨이포인트 헤딩(yaw) 드래그 지정** (2026-08-24). RViz 의 2D Goal Pose 와
      같은 조작 — 누른 자리가 위치, 끈 방향이 헤딩. 안 끌면(8 SVG단위 미만) 0°.
      문턱을 둔 이유: 손떨림 2~3 단위를 헤딩으로 읽으면 클릭마다 무작위 방향이
      박히는데, `yaw_goal_tolerance` 가 11.5° 라 그게 그대로 제약이 된다.
      · yaw 는 **지도 좌표에서** 잰다 — 화면은 y 가 아래로 증가해서 화면 각도를
        그대로 쓰면 위아래가 뒤집힌다. 위로 끌기 = +90° 를 시험으로 고정했다.
      · 핀에 헤딩 화살표를 그리고, 목록에도 각도를 표시한다.
      ⚠ 자세 지정 목표는 여전히 **알려진 실패 모드**다 (control_pipeline.md §12.5.2 —
        목표 0.16 m 앞에서 180° 회전만 하면 되는데 4.05 m 헤매다 실패).
        이제 조작자가 그 상황을 손으로 만들 수 있다. §12.5.2 의 후속안(도킹 접근
        패턴 = 목표 앞 1 m 경유지 + 직진 진입)을 붙이면 완화된다.
- [x] **로봇 마커를 맵 축척에 연동** (2026-08-24). 마커가 픽셀로 그려져 있어 맵이
      바뀌면 차가 실제보다 2배 넘게 커 보였다(실측맵에서 54 SVG단위 = 3.0 m 상당,
      실제 1.37 m). `index.html` 에서 **미터로** 다시 그리고(footprint 는 nav2.yaml
      과 같은 값) `map2d.js` 가 px/m 를 곱한다. 하한 10 px/m — 아주 넓은 맵에서
      차가 4 SVG단위로 뭉개지면 크기의 정확성보다 '어디 있는지 보이는 것'이 우선.
      · 웨이포인트 핀은 **일부러 축척을 안 따른다** — 물리적 물체가 아니라 조작용
        표식이라 넓은 맵에서 몇 픽셀로 줄면 누를 수가 없다(지도 앱의 핀과 같다).
      · 핀 위치는 이제 매 렌더마다 미터 좌표에서 다시 계산한다. 예전에는 클릭
        순간의 화면 좌표를 박아 둬서, 맵을 바꾸면 핀이 엉뚱한 곳에 남았다.
      검증: `alm-webui-v0.6/tools/map-marker-test.mjs` 15항목.
- [ ] ⚠ **웨이포인트 세트 저장/불러오기가 목업** — 불러오기가 맵과 무관한 고정 좌표를
      `state.waypoints` 에 넣는데, Phase 4-2 로 목표 전송이 실경로가 되면서 그게 그대로
      로봇에 나갈 수 있게 됐다. 2026-08-24 에 `disabled` 로 잠갔다(배지만으로는 클릭이
      안 막힌다). 서버에 세트 저장(`/api/waypoints`?)을 붙일 때 푼다.
      ★ 교훈: **옆 경로가 실경로가 되면 무해했던 목업이 함정이 된다.** Phase 5 를 붙일
        때도 수동주행 탭의 남은 목업을 같은 눈으로 다시 볼 것.
- [x] ⭐ **Phase 5 웹 수동주행 — twist 가 아니라 rpm/조향각 직접** (2026-08-24).
      ★ **선행조건 두 개가 사라졌다.** twist 를 안 쓰므로 `##CONFIRM##` 상수 8개가
        확정될 때까지 기다릴 이유가 없어졌다 — 오히려 **그 상수를 측정하는 도구**가
        이것이다(rpm 을 알고 실제 속도를 재면 wheel_radius x gear_ratio 가 나온다).
        경쟁 해소도 토픽 공유가 아니라 `cmd_arbiter` 에 `web` 동작권을 추가해 끝냈다.
      · `alm_msgs/DirectDrive` 신설 (speed_rpm + steer_deg + mode_id)
      · `cmd_arbiter`: OWNERS 에 `web` 추가. web 소유 중에는 `/cmd_vel_mux` 로
        **0 twist 만** 내보내 twist 경로를 닫는다. HELD 시 rpm 만 0 으로 하고
        **조향각은 유지** — 굴러가는 중에 조향을 0 으로 되돌리면 그건 세우는 게
        아니라 조타다.
      · `command_manager._tick_direct`: twist 경로를 통째로 건너뛰되 **물리·안전
        게이트는 전부 유지** (E-STOP 래치 · MCU fault · 명령 타임아웃 · 기동 정렬
        dwell · 모드 전환 dwell · 조향 슬루 · rpm 가감속). 버리는 것은 뜻이 없는
        것들뿐이다(twist->rpm 변환, R_min wz 클램프, auto 선택, ALIGN, odom 워치독).
      · 감속률을 가속과 분리(`direct_rpm_decel` 600 > `direct_rpm_accel` 400).
        twist 경로가 `soft_stop_decel`(1.5) > `max_accel_x`(1.0) 인 것과 같은 이유.
      · backend `/api/manual/*` 5개 + 20 Hz 재발행. **데드맨이 두 겹**이다:
        브라우저→백엔드 0.4 s, 백엔드→arbiter 0.5 s. 탭을 닫아도 로봇이 선다.
      · UI: 키보드 데드맨(W/A/S/D, 스페이스=E-STOP) 추가로 **불일치 #14 해소**.
        요청 rpm 과 `/mcu/command` 실제 rpm 을 나란히 표시한다 — 모드 전환 dwell
        동안 둘이 갈리는데, 안 보여주면 '밀었는데 안 간다' 로만 겪는다.
      · 검증: `alm_base_control/tools/direct_drive_test.py` 16항목 (동작권 게이팅 ·
        클램프 · 슬루 · HELD · E-STOP) 전부 통과. 하드웨어 없이 돈다.
      ⚠ **실차 미검증.** 그리고 이 경로는 **속도를 모른다** — rpm 이 몇 m/s 인지는
        상수가 확정돼야 안다. **잭업 상태에서 먼저 확인할 것.**
- [ ] ⭐ **`##CONFIRM##` 상수 실측** — 이제 도구가 생겼다. 웹 수동주행으로 알려진
      rpm 을 주고 실제 이동거리/시간을 재면 `wheel_radius_m x gear_ratio` 가 나오고,
      알려진 조향각으로 원을 그리면 `max_steer_deg` 와 지오메트리가 검산된다.
      이게 확정되면 twist 경로의 신뢰도가 통째로 올라간다(§3.97 의 여러 항목이 여기 달려 있다).

### 3.95 Nav2 경로계획 재구성 (2026-08-11)
- [x] **Hybrid-A* 전환**. `NavFn+DWB` → `SmacPlannerHybrid(Reeds-Shepp)` +
      `ConstrainedSmoother` + `MPPI`. `dev/sc-lio-sam` 의 `37f524b` 구조를 가져오되
      파라미터는 이 브랜치의 STM32 CONS/URDF 에서 전부 재유도했다.
      2D 격자 플래너가 아니라 Hybrid 를 고른 이유와 맵 타당성 근거 →
      `docs/nav2_planning.md`.
- [x] **조향각 한계**를 `command_manager` 에 구현 (`steer_limit_enabled`).
      `|wz| ≤ |vx|/R_min`, R_min=1.643 m 는 `fourwis_encode.min_turn_radius()` 가
      런타임 계산(후륜 50% 역조향 반영). 가속 램프 통과 후에도 재적용.
- [x] `nav2_kinematic_check.py` — `base_control.yaml` 기구 상수와 `nav2.yaml` 의
      유도값이 어긋나면 알려주는 읽기 전용 검사기. 상수를 바꿀 때마다 돌릴 것.
- [x] `lookup_table_size` 20.0 → 10.0. planner configure 8.19 s → 2.40 s (개발 PC 실측).
- [ ] **실제 경로 생성/주행 미검증.** 지금까지 확인한 것은 "설정이 받아들여지고
      플러그인이 뜬다"까지다. TF·맵·측위가 붙은 전체 스택에서 `/plan` 이 나오는지,
      MPPI 가 그 경로를 따라가는지 확인 필요.
- [ ] Orin Nano 계획시간 측정. `max_planning_time: 2.0` 안에 안 끝나면
      `downsample_costmap: true` + `downsampling_factor: 2` 부터 적용
      (`docs/nav2_planning.md` §4 순서표).
- [ ] ⚠ **저속 사각지대**: `vx < 0.03` 이면서 `|wz| < auto_spin_angular_threshold(0.35)`
      인 구간은 wz 가 0 으로 접히고 spin 도 안 걸린다. MPPI 가 스스로 wz 를 키워
      빠져나오는 것에 기대고 있다. 실차에서 이 구간에 머물면 임계값을 0.20 으로 낮출 것.
- [ ] ⚠ **URDF vs STM32 CONS 지오메트리 불일치**. (2026-08-19: nav2_kinematic_check.py §6 이 이제 이걸 검출한다. 실측 확정만 남음.) URDF 는 휠베이스 0.9116 m /
      윤거 1.0 m (front_x 0.6106, rear_x -0.3010, half_track 0.5), CONS 는
      B=1.0 m / T=0.919 m 로 **사실상 뒤바뀌어 있다.** 둘 다 '실측'이라 주장하므로
      어느 쪽이 맞는지 확인해야 한다. 현재 nav2/조향 제한은 CONS 를 따르고
      footprint 만 URDF 를 따른다 — R_min 과 footprint 가 서로 다른 출처인 셈이다.

### 3.97 제어 파이프라인 보강 (2026-08-19) — docs/control_pipeline.md
- [x] **조향 슬루 제한 + cmd_vel 재정합**. 출발 선회에서 조향 명령이 1500 deg/s 로
      튀던 문제(|wz|<=|vx|/R_min 클램프가 걸리면 R=R_min = 정의상 풀락). 45 deg/s 로
      제한하고, 제한된 조향각에서 wz 를 역산해 McuCommand 두 필드를 정합시킴.
- [x] **기동 조향 정렬 dwell** (자동 1.33 s) — 전원 투입 시 조향각을 모르는 채로 출발하던 문제.
- [x] **모드 전환 dwell** (0.5 s) — normal(±30°)/crab(90°)/spin(47°) 전환 중 조향축 스윕.
- [x] **정지 2등급 분리** — e-stop/fault=즉시, cmd timeout/odom stale=감속 램프.
- [x] **MCU 업링크 부재 경고** — stop_on_mcu_fault 가 실차에서 죽은 안전장치임을 기동 시 경고.
- [x] **fake_mcu 조향 액추에이터 모델** (슬루+지연, 실제 조향각으로 데드레커닝).
      publish_state:=false 로 실차(업링크 미구현) 조건 재현 가능.
- [x] **BT IsPathValid** — 재계획이 순수 시간 구동이던 문제. 경로 무효화 시 즉시 재계획.
- [x] **동적 장애물 지연** 4.30 s/1.94 m → 약 1.44 s/0.65 m
      (global costmap 1→3 Hz, max_planning_time 2.0→1.0, downsample_costmap on,
       through_poses hz 0.333→1.0).
- [x] **죽은 설정 정리** — max_angular_z 0.8→0.45, amcl tf_broadcast→false,
      velocity_smoother CLOSED_LOOP 지뢰 주석.
- [x] **steering_observer 신규** — /Odometry.pose 만으로 κ=Δθ/Δs → 실효 조향각 역산.
      twist 가 0 이어도 동작한다(곡률은 시간이 아니라 거리로 정의되므로). 로깅 전용.
- [x] **nav2_kinematic_check 확장** — §5 조향 응답, §6 URDF↔CONS 대조 추가.
- [x] **속도·슬루율 동시 축소** (0.45 m/s·45 deg/s -> 0.20 m/s·20 deg/s, 비 100 deg/m 유지).
      경로 품질은 조향슬루율/속도 비가 정하고 안전성은 슬루율 절대값이 정한다는 관계를
      이용. 경로 모양은 그대로이면서 S>=20 deg/s 인 모든 액추에이터에서 조향 오차 0.
      MPPI 지평선도 63x0.10 으로 조정해 예측거리 1.26 m 복원. docs/control_pipeline.md §5.12
- [x] ⭐ **S_정지 실측 완료** (2026-08-20, `steer_bench.py`, 접지 상태 -30°→+30° 3회).
      결과 **17.5 / 20.4 / 20.5 deg/s → 최솟값 17.5 채택**. `base_control.yaml` 의
      신규 파라미터 `steer_rate_stopped_deg_s: 17.5` 에 기록.
      ★ **스톨하지 않았다** — '굴리면서만 조향' 이라는 설계 제약은 생기지 않았고
        정지 dwell 로 조향을 펴는 현재 설계가 그대로 유효하다.
      판정: startup 5.0 s → 필요 3.43 s ✔ / **dwell 4.0 s → 필요 4.40 s ❌ (부족)**
            / crab 은 6.86 s 필요(미사용이라 미반영).
      → **`mode_switch_dwell_sec` 4.0 → 5.0** 으로 상향. 아래 항목 참고.
      → `nav2_kinematic_check.py` §5 가 이제 vx=0 dwell 을 `max_steer_rate_deg_s`(주행 중)가
        아니라 `steer_rate_stopped_deg_s`(정지)로 나눠 검사한다. **이게 4.0 미달의 원인이었다.**
- [ ] ⚠ **주행 중 슬루율 S 실측** — `max_steer_rate_deg_s: 20.0` 은 여전히 **가정값**이다.
      급하지는 않다: 실측 `S_정지 = 17.5` 가 하한 근사이고(굴러갈 때가 더 빠름)
      20 은 그 하한을 14% 웃돌 뿐이다. 출발 직후(`vx≈0.03`)에만 명령이 잠깐 앞선다.
      측정법: `steering_observer` 상한탐색 / 폰 슬로모 240fps / 자이로 스텝응답.
      실측하면 `max_linear_x` 와 **비(100 deg/m)를 유지한 채 함께** 올려 성능 회복.
- [ ] **모드 전환 스윕 직접 실측** — dwell 5.0 s 는 `77° ÷ 17.5` 계산값이다.
      `steer_bench.py` 는 normal 범위(±30°) 안에서만 쟀으므로 spin 자세(47°)까지는 외삽.
      세워두고 `mode_id` 만 바꿔 바퀴가 새 자세로 다 돌아가는 시간을 직접 재면 외삽이 없어진다.
      ALIGN 1회 고정비 10 s 가 여기 달려 있다.
- [ ] ⚠ **STM32 업링크 요청** — msg_type 0x03 "MiniState" 11 B 제안
      (status_flags 1 + fault_code 2 + steer_actual 4 + ack_seq 4).
      질문 Q1~Q6(액추에이터 종류/내부 램프 여부/슬루율/mode 0 거동/CONS(4)) 은
      docs/control_pipeline.md §7.2.
- [x] **시뮬레이션 통합 검증** (docs/control_pipeline.md §12). 실측 SLAM 맵(alm_lab)에서
      실제 파라미터/BT 그대로, 하드웨어만 대역(sim_world + fake_mcu)으로 전 구간 주행.
      · 조향 슬루 제한·cmd_vel 재정합·기동정렬·모드dwell 전부 실측 확인 (정합오차 2.6e-9 rad/s,
        명령 vs 실제 조향 간극 평균 0.010°)
      · **충돌 0** (전 시험 통틀어)
      · 검증 과정에서 **내가 심은 결함 3개**를 잡음:
          ① BT 재계획이 1 Hz 아닌 18.4 Hz (RateController halt 시 타이머 리셋)
             -> PathExpiringTimer 로 교체
          ② auto_mode_min_hold_sec 4.0 상향이 협착 목표 회귀 유발 -> 0.80 으로 되돌림
          ③ steering_observer 가 직진(κ=0)에서 ZeroDivisionError -> 가드 추가
      · 재계획 주기 1 Hz -> 3 Hz (전역 costmap 갱신률과 짝. 근거 §12.5.1)
- [x] **ALIGN — 경로 헤딩 정렬 기동 신규** (2026-08-20). 아래 후보 (c) 를 구현한 것이다.
      `scripts/path_align.py`(신규) + `command_manager` 가 **`/plan` 을 직접 구독**해
      경로가 요구하는 헤딩 대비 오차가 60° 넘게(0.6 s 지속) 벌어지면 목표 헤딩을
      **절대각으로 래치**하고 spin 으로 정정 후 normal 복귀(이탈 15° · 쿨다운 3 s).
      기존 spin 진입 조건(`|wz|>=0.35 and lin<=0.04`)은 Nav2 twist 의 **사후 분류**라
      플래너가 제자리 회전을 안 내는 이상 BT 리커버리 때만 발동했다 — ALIGN 이
      4WIS 의 제자리 회전을 **선제적으로** 쓰는 유일한 경로다.
      · 자체 시험 19항목 (`python3 path_align.py`), 정합성 검사 §7 추가
      · 실측: alm_lab 640 s 중 1회 발동 (-62.4° -> -14.9°, 5.2 s), 왕복 없음
      · A/B (개활 2회·협착 1회): 발동 안 하는 목표는 숫자가 소수점까지 동일 = **회귀 0**
      · 문턱 60° 는 실측 분포 근거 (경로 추종 중 헤딩오차 p90 9.7°, 30°로 낮추면 틱의 35%)
      상세 docs/control_pipeline.md §6.8 · §7.3
- [x] **mode_switch_dwell_sec 3.0 -> 4.0 -> 5.0** (2026-08-20). 기구학 미달이었다:
      최악 스윕 normal 풀락(30°) -> spin(47°) = 77°.
      1차(4.0)는 ÷20 deg/s = 3.85 s 로 잡았는데, **20 은 '주행 중' 슬루율**이고
      이 dwell 은 vx=0 구간이라 나눌 값이 틀렸다. 실측 `S_정지 17.5` 로 다시 나누면
      **4.40 s** 라 4.0 은 여전히 부족했다.
      2차(5.0)는 여유를 붙인 값 = 77° ÷ 15.4, 즉 S 가 12% 떨어져도 스윕을 덮는다.
      조향각 업링크가 없어 **미달을 관측할 방법이 없으므로** 여유를 산다.
      비용: ALIGN 1회당 2×dwell = 10 s 정지 고정비(실측 진입빈도 264 s 에 2회 ≈ 7.6%).
      ⚠ `auto_mode_min_hold_sec` 로 상쇄하려 들지 말 것 — 4.0 상향은 회귀였다.
      ⚠ crab(90°) 은 6.86 s 필요 — 수동 crab 을 쓸 거면 7.0 으로 올릴 것.
      `nav2_kinematic_check` §5 가 `steer_rate_stopped_deg_s` 로 검사한다.
- [ ] ⚠ **경로가 아예 안 나오는 목표** (ALIGN 으로도 못 고침, 2026-08-20 실측).
      막다른 포켓형 목표에서 `replans: 0` — `/plan` 이 한 번도 발행되지 않는다.
      ALIGN 은 경로가 입력이라 개입할 수 없다.
      ★ **원인은 헤딩이 아니다.** 시작 헤딩 0°/-45°/-90° 전부 `no valid path found`.
        리커버리 spin 이 실제로 0°->96°->187° 로 돌려놨는데도 계획은 계속 실패했다.
      의심 순서: ① `inflation_radius` 1.2 m vs 포켓 폭 2.8 m (중앙조차 팽창비용 높음)
                 ② `max_planning_time` 1.0 s  ③ Hybrid-A* 각도 이산화
      재현: 협착 ㄱ자 합성맵 (scratchpad `make_lmap.py`, R_min 유턴 가능 셀 0개)
- [ ] 🔬 **하네스 실행편차 — 단일 실행 A/B 는 못 믿는다** (2026-08-20).
      같은 목표가 실행마다 323 s -> ABORT -> 163 s -> TIMEOUT 로 흔들린다.
      **ALIGN 이 0회 발동한 실행에서도** 좌우가 갈렸다 = 전부 잡음이다.
      원인: MPPI 가 매 틱 1600 궤적을 무작위 표본으로 굴리고, 시뮬이 벽시계로 돈다.
      ##TODO## 위 §3.97 의 "되돌림" 판정 중 성공/실패로 갈린 둘
      (`auto_mode_min_hold_sec 4.0`, `spin 문턱 0.10`)은 **재검증 대상**이다.
      반복 실행 스크립트: scratchpad `run_ab.sh` (`ALIGN` 토글, `REPS` 반복).
      판단은 성공/실패가 아니라 연속량(소요시간·자세오차·우회율)으로 할 것.
- [x] **`goal_yaw_aligner.py` 삭제** (2026-08-20). 어떤 런치도 띄우지 않고 있었고,
      실측에서 한 번도 발동하지 않았으며, ALIGN 이 액션 선점 없이 같은 일을 더 앞에서 한다.
      회전을 명령하는 주체가 둘인 구조 자체가 위험해서 제거했다.
- [~] ⚠ **플래너가 제자리 회전을 표현하지 못한다** (검증으로 새로 드러난 구조적 공백).
      ※ 2026-08-20: 이 설명은 **불완전했다.** 제자리 회전을 쓸 수 있게 해도(리커버리
        spin 이 96°·187° 로 실제로 돌려놨다) 계획은 여전히 실패했다. 위 '경로가 아예
        안 나오는 목표' 항목 참고. ALIGN 은 '경로가 있는데 헤딩만 어긋난' 경우를 덮는다.
      해결 시도 2건 모두 되돌림 — docs/control_pipeline.md §8.5, §8.6:
        · SmacPlannerLattice(회전 프리미티브) 도입 -> 개활은 무회귀였으나 하드케이스 악화
          (control set 은 alm_navigation/lattice_primitives/ 에 검증된 채로 보존)
        · spin 진입 문턱 0.35->0.10 -> G3 못 고치고 G4 회귀
      ##TODO## 다음 후보: (a) TightSpace 계획 실패율 규명  (b) 도킹 접근 패턴
      (목표 앞 1 m 경유지 + 직진 진입, 클라이언트 레벨)  (c) spin 탈출 조건을
      요레이트 기반 -> 헤딩오차 기반으로.
      SmacPlannerHybrid 는 minimum_turning_radius=1.643 m 프리미티브로만 탐색하므로
      '가까운 거리에서 큰 자세변화'에 해가 없다 -> 빈 경로 반환 -> 리커버리 소진 -> abort.
      실측: 목표 0.16 m 앞에서 180deg 회전만 하면 되는데 4.05 m 헤매다 실패(우회 24.6배).
      **플랫폼에 spin 이 있는데 플래너가 쓸 수 없다** (MPPI 가 우연히 낼 때만 발동).
      · 자유 주행(자세 무관)에는 지장 없음 — G1/G2/G4/Y1/Y3 전부 성공
      · 도킹 등 **자세 지정 목표**를 쓸 계획이면 대책 필요
      · 유력안: BT 에 '위치 먼저 -> 도착 후 Spin 으로 자세 정렬' 단계 추가
      · crab 은 해법이 아님(회전을 못 만듦). 상세 docs/control_pipeline.md §12.5.2
- [ ] **실차 주행 미검증** — 위는 전부 시뮬레이션까지만이다.
      가정: 조향 슬루율 60 deg/s(임의값) · 측위 오차 0 · 슬립 없음 · 정적 장애물만 ·
      개발 PC 8코어. Orin Nano CPU 여유 미확인.

### 3.98 리뷰 반영 + 워크스페이스 전수 점검 (2026-08-20) — docs/control_pipeline.md §6.9·§6.10
- [x] **후진 조향 부호 반대** 수정 (기존 결함, Blocker).
      `ω = v/R` 이므로 같은 조향각에서 후진하면 요레이트가 뒤집힌다. 인코더가
      `wz` 부호만 봐서 후진 선회가 통째로 반대로 돌았다. `wz_from_steer`(역함수)도
      **같이 틀려 있어서** '인코딩→역산' 자체시험이 이를 통과시켰다.
      -> 코드를 참조하지 않고 자전거 모델로만 기대부호를 세우는 `[sign]` 시험 추가.
      Hybrid-A* 가 후진을 실제로 쓰므로(reverse_penalty 3.0) 실차 영향이 컸다.
- [x] **조향 clamp 시 전진속도 1.64배 부풀림** 수정 (리뷰에 없던 것, 위 수정 중 발견).
      요청 반경 < R_min 일 때 내측전륜 속도를 `wz` 기준으로 계산해, 못 도는 것을
      '더 빨리 가는' 것으로 보상하고 있었다. -> `vx` 기준으로 변경(비클램프 구간은 동일).
- [x] **직선 후진을 180° 오정렬로 판단** 수정 (ALIGN 의 새 결함).
      'yaw 가 안 변하면 미채움' 판정이 틀렸다 — 직선 경로는 전/후진 모두 yaw 가 상수다.
      -> 판정을 **'yaw 가 진행 방향과 정합하는가'** 로 교체. 미채움이면 추측하지 않고
      ALIGN 을 쉬게 둔다(틀린 추측의 대가가 불필요한 180° 회전이라 너무 비싸다).
- [x] **슬루 후 speed_rpm 재계산** (`rpm_for_steer`). 최대 -15.8% 차체속도 오차였다.
- [x] **MPPI model_dt == 제어주기** (0.10x63 -> 0.05x126, 20 Hz 유지).
      `Control sequence shifting is ON` 확인. ⚠ `controller_frequency` 10 Hz 안은
      BT 경고 0->815, 3목표 전패로 **되돌림**(control_pipeline.md §8.10).
- [x] **expected_planner_frequency** 1.0 -> 2.5 (실측 달성률 2.92~2.96 Hz 반영).
- [x] **local_costmap update_frequency** 5 -> 10 Hz (제어 20 Hz 대비 코스트맵 지연 절반).
- [x] **후방 사각지대 기동 경고** 추가. 자기가림 마스크(180±30°, 1.5 m)는
      `/livox/lidar` 자체를 깎아 FAST-LIO·ICP·costmap·/scan 에 동시 적용된다 =
      **아무것도 안 보이는 영역**이다. 값은 안 줄였다(줄이면 적재물이 정합에 섞여
      측위가 흔들린다). 완화: 그 영역은 레이캐스팅도 없어 **clearing 이 안 되므로**
      등지기 전에 본 장애물은 남는다. 못 보는 것은 '등진 뒤 새로 나타난 물체'.
- [x] **`goal_yaw_aligner.py` 삭제** — 위 §5 참고.
- [ ] ⚠⭐ **LiDAR 마운트 TF 실측** — `lidar_x/y/z` + roll/pitch/yaw 가 전부 추정값
      (`0, 0, 0.5`, 회전 0). FAST-LIO 의 odom->base_link · costmap 장애물 배치 ·
      /scan z밴드 **세 곳에 동시에** 들어간다.
      ★ '초기위치가 잘 잡힌다'로는 안 잡힌다 — 맵도 같은 TF 로 만들었으면 매핑과
        측위가 일관되게 같이 틀리므로 ICP 는 잘 수렴한다. 어긋나는 것은
        **맵과 실제 차체의 관계**이고 그건 footprint 충돌검사에 그대로 들어간다.
      **줄자로 재서 넣을 것.** roll/pitch 도(MID-360 은 기울여 다는 경우가 많다).
- [ ] 후방 사각지대 운용 결정 — 마스크를 적재물 최소 크기로 줄일지.
      `lidar.launch.py mask_debug:=/livox/lidar_masked` 로 잘려나간 점을 눈으로 보고 정할 것.
- [ ] `reverse_penalty: 3.0` 재검토 — 지금 값은 **시간 비용**(전/후진 속도비 2.86)에서
      나왔고 후방이 사각지대라는 **위험 비용**은 안 들어가 있다.

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
