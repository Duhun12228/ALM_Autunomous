# ALM 4WIS 모션 프리미티브 (SmacPlannerLattice 용)

`alm_1.643m_diff.json` — 이 플랫폼 전용 control set.

## 왜 필요한가

`SmacPlannerHybrid` 는 `minimum_turning_radius` 호(arc)만으로 탐색하므로
**제자리 회전을 표현할 수 없다.** 그래서 "좁은 곳에서 방향을 되돌려야 하는"
경로에는 해가 없고, 플래너가 빈 경로를 낸다.

그런데 이 로봇은 4륜 독립조향이라 `spin` 으로 제자리 회전이 된다.
`SmacPlannerLattice` 는 control set 에 제자리회전 프리미티브가 있으면 그걸
탐색에 쓰고, `rotation_penalty` 로 "꼭 필요할 때만" 쓰게 조절할 수 있다.

실제 맵(alm_lab)에서 이게 얼마나 차이나는지:

| | 면적 | 연결성분 | 최대 조각 |
|---|---|---|---|
| 로봇이 들어갈 수 있는 곳 | 1429.7 m² | 19 | 99.9% |
| 제자리 회전 가능 | 1297.9 m² | 9 | **96.4%** |
| R_min U턴 가능 | 956.9 m² | 6 | **50.7%** |

통행가능 영역의 **33.1%** 에서는 R_min 만으로 방향을 되돌릴 수 없다.

## 어떻게 만들었나

nav2 공식 생성기(`nav2_smac_planner/lattice_primitives`, humble 브랜치)로 구웠다.

```bash
pip install rtree
python3 generate_motion_primitives.py --config config_alm.json --output alm_1.643m_diff.json
```

`config_alm.json`:
```json
{
    "motion_model": "diff",
    "turning_radius": 1.6425,
    "grid_resolution": 0.05,
    "stopping_threshold": 5,
    "num_of_headings": 16
}
```

- `motion_model: "diff"` — **제자리회전 프리미티브를 포함시키려고** diff 를 쓴다.
  `ackermann` 세트에는 회전이 0개다. diff 라고 해서 로봇이 차동구동이 되는 게
  아니라, "R_min 호 + 제자리회전" 이라는 이 로봇의 실제 능력 집합과 일치한다.
- `turning_radius: 1.6425` — `fourwis_encode.min_turn_radius()` 값.
  기본 제공 세트(0.5 m / 1 m)를 쓰면 **로봇이 못 도는 반경의 경로**가 나온다.

## 검증

```
프리미티브 408개 (제자리회전 32개)
최소 선회반경 1.677 m >= R_min 1.6425   <- 기구 한계 준수
grid_resolution 0.05 = costmap resolution
num_of_headings 16 (22.5°/bin)
```

## 다시 구워야 할 때

`base_control.yaml` 의 `wheelbase_m`/`track_m`/`rws_ratio`/`max_steer_deg` 를
바꾸면 R_min 이 바뀌므로 **이 파일도 다시 구워야 한다.**
`nav2_kinematic_check.py` 가 `lattice_metadata.turning_radius` 와 R_min 을 대조한다.
