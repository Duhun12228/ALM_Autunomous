#!/usr/bin/env python3
"""nav2.yaml 이 base_control.yaml 의 기구 상수와 어긋나지 않는지 검사한다.

    ros2 run alm_navigation nav2_kinematic_check.py

왜 필요한가
-----------
Nav2 의 몇몇 파라미터는 base_control 의 기구 상수에서 **유도된 값**이다.
minimum_turning_radius 가 대표적이다 — wheelbase/track/rws_ratio/max_steer_deg 를
하나라도 바꾸면 R_min 이 바뀌는데, nav2.yaml 은 그 사실을 알 방법이 없다.

어긋나면 조용히 나빠진다. 플래너는 로봇이 못 도는 경로를 내고, command_manager 는
조향각을 포화시키며, 실제 궤적이 계획과 갈라진다. 어느 로그에도 '어긋났다'고
찍히지 않는다. 그래서 숫자를 직접 비교한다.

이 스크립트는 **읽기 전용**이다. 고쳐주지 않고 어디가 어긋났는지만 알려준다.
어느 쪽이 맞는지는 사람이 정해야 한다.

종료코드: 0 = 전부 일치, 1 = 어긋남 있음, 2 = 파일/의존성 문제
"""

import math
import os
import re
import sys

try:
    import yaml
except ImportError:
    print("PyYAML 이 없습니다: pip3 install pyyaml", file=sys.stderr)
    sys.exit(2)

try:
    from ament_index_python.packages import (get_package_prefix,
                                             get_package_share_directory)
except ImportError:                                          # pragma: no cover
    get_package_prefix = get_package_share_directory = None

# 빌드 전에도 돌아야 한다. 상수가 어긋났는지 확인하고 싶은 시점은 대개
# '고치고 나서 빌드하기 직전'이지, 빌드가 끝난 뒤가 아니다. 그래서 설치본을
# 먼저 찾고, 없으면 이 스크립트 위치를 기준으로 소스 트리에서 찾는다.
#   <ws>/src/alm_navigation/scripts/nav2_kinematic_check.py  ->  <ws>/src
_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _share(pkg, *rel):
    """설치본 share 우선, 없으면 소스 트리에서 찾는다."""
    if get_package_share_directory is not None:
        try:
            path = os.path.join(get_package_share_directory(pkg), *rel)
            if os.path.exists(path):
                return path, "설치본"
        except Exception:
            pass
    path = os.path.join(_SRC, pkg, *rel)
    return (path, "소스") if os.path.exists(path) else (None, None)


def _import_fourwis():
    """command_manager 와 '같은' 모듈을 쓴다 — 복사하면 그 순간 갈라진다."""
    cands = []
    if get_package_prefix is not None:
        try:
            cands.append(os.path.join(
                get_package_prefix("alm_base_control"), "lib", "alm_base_control"))
        except Exception:
            pass
    cands.append(os.path.join(_SRC, "alm_base_control", "scripts"))
    for d in cands:
        if os.path.exists(os.path.join(d, "fourwis_encode.py")):
            sys.path.insert(0, d)
            import fourwis_encode as mod
            return mod, d
    return None, None


fourwis_encode, _FW_DIR = _import_fourwis()
if fourwis_encode is None:                                   # pragma: no cover
    print("fourwis_encode.py 를 찾지 못했습니다 "
          "(alm_base_control/scripts 또는 설치본 lib).", file=sys.stderr)
    sys.exit(2)


GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
if not sys.stdout.isatty():
    GREEN = RED = YELLOW = DIM = RESET = ""

_problems = []


def _load(path):
    with open(path, "r", encoding="utf-8") as fp:
        return yaml.safe_load(fp)


def _params(doc, node, *nested):
    """ROS 파라미터 파일에서 <node>[/<nested>...]/ros__parameters 를 꺼낸다."""
    cur = doc.get(node, {}) or {}
    for key in nested:
        cur = cur.get(key, {}) or {}
    return cur.get("ros__parameters", {}) or {}


def check(label, actual, expected, tol=1e-3, hint=""):
    ok = actual is not None and abs(float(actual) - float(expected)) <= tol
    mark = f"{GREEN}OK  {RESET}" if ok else f"{RED}불일치{RESET}"
    shown = "없음" if actual is None else f"{float(actual):.4g}"
    print(f"  [{mark}] {label:44s} nav2={shown:>10s}  기대={float(expected):.4g}")
    if not ok:
        _problems.append((label, shown, f"{float(expected):.4g}", hint))
    return ok


def check_ge(label, actual, floor, hint=""):
    ok = actual is not None and float(actual) > float(floor)
    mark = f"{GREEN}OK  {RESET}" if ok else f"{RED}불일치{RESET}"
    shown = "없음" if actual is None else f"{float(actual):.4g}"
    print(f"  [{mark}] {label:44s} nav2={shown:>10s}  요구> {float(floor):.4g}")
    if not ok:
        _problems.append((label, shown, f"> {float(floor):.4g}", hint))
    return ok


def note(label, value, hint=""):
    print(f"  [{YELLOW}참고{RESET}] {label:44s} {value}")
    if hint:
        print(f"         {DIM}{hint}{RESET}")


def check_present(label, value, hint=""):
    """값이 선언돼 있기만 하면 통과 (기대값이 아직 없는 ##CONFIRM## 항목용)."""
    ok = value is not None
    mark = f"{GREEN}OK  {RESET}" if ok else f"{RED}없음  {RESET}"
    shown = "없음" if value is None else f"{value}"
    print(f"  [{mark}] {label:44s}      ={shown:>10s}")
    if not ok:
        _problems.append((label, "없음", "선언 필요", hint))
    return ok


def _xacro_props(path):
    """URDF xacro 의 <xacro:property name=".." value=".."/> 를 dict 로.

    URDF 를 완전히 파싱하지 않는 이유: xacro 전개에는 xacro 패키지가 필요하고,
    이 검사기는 '빌드 전에도 돌아야 한다'는 전제가 있다. 여기서 필요한 것은
    최상단 상수 몇 개뿐이라 정규식으로 충분하다.
    """
    props = {}
    try:
        with open(path, "r", encoding="utf-8") as fp:
            text = fp.read()
    except OSError:
        return props
    for name, value in re.findall(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"', text):
        try:
            props[name] = float(value)
        except ValueError:
            pass                      # ${...} 같은 식은 건너뛴다
    return props


def main():
    bc_path, bc_src = _share("alm_base_control", "config", "base_control.yaml")
    nav_path, nav_src = _share("alm_navigation", "config", "nav2.yaml")
    if bc_path is None or nav_path is None:
        print("base_control.yaml / nav2.yaml 을 찾지 못했습니다.", file=sys.stderr)
        return 2

    bc = _params(_load(bc_path), "command_manager")
    nav_doc = _load(nav_path)

    # ---- 기구 상수에서 파생값 계산 (command_manager 와 동일한 코드 경로) ----
    wis = fourwis_encode.FourWISParams(
        wheelbase_m=bc.get("wheelbase_m", 1.0),
        track_m=bc.get("track_m", 0.919),
        rws_ratio=bc.get("rws_ratio", 0.5),
        wheel_radius_m=bc.get("wheel_radius_m", 0.103),
        gear_ratio=bc.get("gear_ratio", 1.0),
        max_steer_deg=bc.get("max_steer_deg", 30.0),
        straight_angle_deg=bc.get("straight_angle_deg", 2.0),
        crab_rpm_scale=bc.get("crab_rpm_scale", 0.5),
        zero_turn_rpm_scale=bc.get("zero_turn_rpm_scale", 0.6),
        max_rpm=bc.get("max_rpm", 3000.0),
    )
    r_min = fourwis_encode.min_turn_radius(wis)
    max_lx = float(bc.get("max_linear_x", 0.45))
    min_lx = float(bc.get("min_linear_x", -0.15))
    wz_normal = fourwis_encode.max_angular_speed(wis, max_lx)
    spin_max_wz = float(bc.get("auto_spin_max_angular_speed", 0.45))

    print(f"\n{DIM}base_control.yaml{RESET}  [{bc_src}] {bc_path}")
    print(f"{DIM}nav2.yaml        {RESET}  [{nav_src}] {nav_path}")
    print(f"{DIM}fourwis_encode   {RESET}  {_FW_DIR}")
    if "소스" in (bc_src, nav_src):
        print(f"{YELLOW}  ※ 설치본이 없어 소스 트리를 읽었습니다. 빌드 후 다시 확인하세요.{RESET}")
    print("\n" + "=" * 78)
    print("기구 상수 (base_control.yaml = STM32 ALM07.slx CONS)")
    print("=" * 78)
    print(f"  wheelbase B = {wis.B:.4f} m   track T = {wis.half_track * 2:.4f} m   "
          f"rws_ratio = {wis.rws:.2f}   max_steer = {math.degrees(wis.max_steer_rad):.1f}°")
    print(f"  -> 최소 선회반경 R_min      = {r_min:.4f} m")
    print(f"  -> normal 최대 요레이트     = {wz_normal:.4f} rad/s  @ vx={max_lx}")
    print(f"  -> spin 최대 요레이트       = {spin_max_wz:.4f} rad/s")

    # ---- 1) 플래너/스무더의 최소 선회반경 ----
    print("\n" + "=" * 78)
    print("1) 최소 선회반경 — 계획이 실현가능한가")
    print("=" * 78)
    planner = _params(nav_doc, "planner_server").get("GridBased", {}) or {}
    smoother = _params(nav_doc, "smoother_server").get("SmoothPath", {}) or {}
    plugin = str(planner.get("plugin", "?"))
    note("planner plugin", plugin)
    if "Hybrid" in plugin or "Lattice" in plugin:
        check("planner.minimum_turning_radius",
              planner.get("minimum_turning_radius"), r_min, tol=0.01,
              hint="Hybrid-A* 가 이 반경으로 프리미티브를 만든다. 크면 못 도는 "
                   "경로가 나오고, 작으면 갈 수 있는 길을 스스로 막는다.")
        exp_len = 2.0 * r_min
        ael = planner.get("analytic_expansion_max_length")
        ok = ael is not None and exp_len * 0.8 <= float(ael) <= exp_len * 1.5
        print(f"  [{GREEN + 'OK  ' + RESET if ok else YELLOW + '확인  ' + RESET}] "
              f"{'planner.analytic_expansion_max_length':44s} "
              f"nav2={float(ael) if ael is not None else float('nan'):.4g}  "
              f"권장≈{exp_len:.4g} (2*R_min)")
    else:
        note("planner 가 Hybrid/Lattice 계열이 아님", plugin,
             "격자 플래너는 R_min 개념이 없어 스무더가 사후에 펴야 한다.")

    # TightSpace(Lattice) 폴백 플래너의 control set 검증
    tight = _params(nav_doc, "planner_server").get("TightSpace", {}) or {}
    if tight:
        note("폴백 planner", str(tight.get("plugin", "?")))
        lat_path, lat_src = _share("alm_navigation", "lattice_primitives",
                                   "alm_1.643m_diff.json")
        if lat_path is None:
            _problems.append(("lattice control set", "없음", "생성 필요",
                              "alm_navigation/lattice_primitives/README.md 참고"))
            print(f"  [{RED}없음{RESET}] {'lattice control set 파일':44s}")
        else:
            import json as _json
            try:
                lat = _json.load(open(lat_path, encoding="utf-8"))
                md = lat.get("lattice_metadata", {})
                prims = lat.get("primitives", [])
                rots = [q for q in prims
                        if q.get("poses") and abs(q["poses"][-1][0]) < 1e-6
                        and abs(q["poses"][-1][1]) < 1e-6]
                note(f"lattice [{lat_src}]",
                     f"{len(prims)}개 프리미티브, 제자리회전 {len(rots)}개")
                check("lattice.turning_radius", md.get("turning_radius"), r_min,
                      tol=0.01,
                      hint="control set 의 선회반경이 R_min 과 다르면 로봇이 못 도는 "
                           "경로가 나온다. 기구 상수를 바꿨으면 다시 구울 것 "
                           "(lattice_primitives/README.md).")
                radii = [q["trajectory_radius"] for q in prims
                         if q.get("trajectory_radius", 0) > 0]
                if radii:
                    check_ge("lattice 최소 실제 선회반경", min(radii), r_min - 1e-6,
                             hint="프리미티브 중 R_min 보다 급한 호가 있으면 안 된다.")
                if not rots:
                    _problems.append(("lattice 제자리회전", "0개", ">0",
                                      "회전이 없으면 TightSpace 를 쓸 이유가 없다 "
                                      "(motion_model 을 diff 로 구웠는지 확인)."))
                    print(f"  [{RED}불일치{RESET}] {'lattice 제자리회전 프리미티브':44s} 0개")
                cm_res = (_params(nav_doc, "global_costmap", "global_costmap")
                          .get("resolution"))
                if cm_res is not None:
                    check("lattice.grid_resolution", md.get("grid_resolution"),
                          cm_res, tol=1e-6,
                          hint="control set 격자와 costmap 해상도가 달라선 안 된다.")
            except Exception as exc:                      # pragma: no cover
                note("lattice 파일 읽기 실패", str(exc)[:60])
        note("rotation_penalty", str(tight.get("rotation_penalty")),
             "제자리회전을 얼마나 꺼릴지. ↑이면 우회 선호, ↓이면 자주 돈다.")

    check("smoother.minimum_turning_radius",
          smoother.get("minimum_turning_radius"), r_min, tol=0.01,
          hint="플래너와 다르면 스무더가 플래너의 결정을 되돌린다. 두 값은 같아야 한다.")

    # ---- 2) 속도 한계 ----
    print("\n" + "=" * 78)
    print("2) 속도 한계 — Nav2 가 base_control 이 허용하는 범위 안에서 계획하는가")
    print("=" * 78)
    fp = _params(nav_doc, "controller_server").get("FollowPath", {}) or {}
    check("MPPI.vx_max", fp.get("vx_max"), max_lx,
          hint="base_control 의 max_linear_x 와 같아야 한다.")
    check("MPPI.vx_min", fp.get("vx_min"), min_lx,
          hint="base_control 의 min_linear_x 와 같아야 한다.")
    check("MPPI.wz_max", fp.get("wz_max"), spin_max_wz,
          hint="spin 모드 상한(auto_spin_max_angular_speed)에 맞춘다. normal 로 "
               "낼 수 있는 값은 이보다 작지만(vx/R_min), 그 구간은 "
               "command_manager 의 조향 클램프가 처리한다.")
    # base_control 의 max_angular_z 는 예전에 검사 대상이 아니었다. 0.8 로 남아
    # 있었는데 어디서도 도달할 수 없는 죽은 값이었다 — 읽는 사람만 헷갈린다.
    check("base_control.max_angular_z", bc.get("max_angular_z"), spin_max_wz,
          hint="command_manager 의 wz 클램프. spin 상한과 같은 값이어야 한다. "
               "이보다 크게 두면 도달 불가능한 값이라 혼동만 만든다.")
    note("normal 모드 실제 상한", f"{wz_normal:.4f} rad/s @ vx={max_lx}",
         f"MPPI wz_max({fp.get('wz_max')})와의 차이가 그대로 클램프된다.")

    # ---- MPPI 운동모델이 이 기구를 반영하는가 (2026-08-23 추가) ----
    # DiffDrive 로 두면 MPPI 는 제자리 회전이 되는 로봇으로 알고 롤아웃한다.
    # 그런데 normal 모드가 실제로 내는 wz 는 vx/R_min 뿐이라, 그 간극만큼
    # 명령이 조용히 잘리고 MPPI 는 그 사실을 모른다 -> 좌우 진동.
    mm = fp.get("motion_model")
    if mm == "Ackermann":
        ack = fp.get("AckermannConstraints", {}) or {}
        check("MPPI.AckermannConstraints.min_turning_r", ack.get("min_turning_r"), r_min,
              tol=0.01,
              hint="Ackermann 모델은 |wz| <= |vx|/min_turning_r 로 롤아웃을 조인다. "
                   "이 값이 R_min 과 다르면 MPPI 의 모델과 실제 기구가 또 어긋난다.")
        note("MPPI motion_model", "Ackermann",
             f"롤아웃이 R_min 을 지키므로 command_manager 클램프로 잘려나가는 "
             f"명령이 없다. 제자리 회전은 ALIGN 과 BT 리커버리가 담당한다.")
    else:
        _problems.append((
            "MPPI.motion_model", str(mm), "Ackermann",
            f"DiffDrive 면 MPPI 가 wz 를 최대 {fp.get('wz_max')}까지 낼 수 있다고 "
            f"믿지만 normal 모드 실제 상한은 {wz_normal:.3f} 이다 "
            f"({(float(fp.get('wz_max') or 0) / max(wz_normal, 1e-9)):.1f}배). "
            f"그 차이가 조용히 클램프되어 좌우 진동이 된다."))

    # ---- spin 진입 문턱이 도달 가능한가 (2026-08-23 추가) ----
    # ##함정## spin 상한을 낮추면서 이 문턱을 안 내리면, 문턱이 상한보다 커져
    #   auto 라우팅이 spin 에 **영영 진입하지 못한다.** 그러면 BT 리커버리
    #   Spin 이 normal 모드로 실행돼 조향 선회가 나간다 — 조용히 망가진다.
    spin_thr = bc.get("auto_spin_angular_threshold")
    if spin_thr is not None:
        if float(spin_thr) >= float(spin_max_wz):
            _problems.append((
                "auto_spin_angular_threshold", str(spin_thr),
                f"< {spin_max_wz} (spin 상한)",
                "문턱이 상한 이상이면 도달할 수 없어 spin 이 영영 안 걸린다. "
                "BT 리커버리 Spin 이 normal 모드로 실행된다."))
        else:
            note("spin 진입 문턱 / 상한",
                 f"{spin_thr} / {spin_max_wz} (비 {float(spin_thr)/float(spin_max_wz):.2f})",
                 "상한을 바꾸면 이 비를 유지한 채 문턱도 같이 옮길 것.")

    # ---- BT 리커버리 Spin 속도 (2026-08-23 추가) ----
    bs = _params(nav_doc, "behavior_server")
    if bs:
        check("behavior_server.max_rotational_vel", bs.get("max_rotational_vel"),
              spin_max_wz,
              hint="BT 리커버리 Spin 의 회전 속도. 여기만 안 내리면 "
                   "velocity_smoother 가 잘라내어 명령과 실제가 또 어긋난다.")
        mrv = bs.get("min_rotational_vel")
        if mrv is not None and float(mrv) >= 0.5 * float(spin_max_wz):
            _problems.append((
                "behavior_server.min_rotational_vel", str(mrv),
                f"<= {0.5 * float(spin_max_wz):.2f}",
                "최소가 최대의 절반을 넘으면 사실상 정속 회전이 되어 "
                "감속 없이 멈춘다."))

    vs = _params(nav_doc, "velocity_smoother")
    mv, mnv = vs.get("max_velocity") or [], vs.get("min_velocity") or []
    if len(mv) == 3 and len(mnv) == 3:
        check("velocity_smoother.max_velocity[vx]", mv[0], max_lx)
        check("velocity_smoother.min_velocity[vx]", mnv[0], min_lx)
        check("velocity_smoother.max_velocity[wz]", mv[2], spin_max_wz)
    # ---- 각가속 3곳 일치 (2026-08-25 추가) ----
    # ##함정## 속도(wz_max)는 네 곳을 맞춰 놓고 **각가속만 velocity_smoother 에
    #   1.5 로 남아 있었다.** 더 빡빡한 0.8 이 뒤에서 이기므로 거동은 같았고,
    #   그래서 아무도 못 봤다. 검사기가 각가속을 아예 안 보고 있었기 때문이다.
    #   이런 건 '조용히 다른 값' 이라 다음 튜닝 때 잘못된 전제로 쓰인다.
    accel_th = bc.get("max_accel_theta")
    if accel_th is not None:
        acc = vs.get("max_accel") or []
        dec = vs.get("max_decel") or []
        if len(acc) == 3:
            check("velocity_smoother.max_accel[wz]", acc[2], float(accel_th),
                  hint="base_control 의 max_accel_theta 와 같아야 한다. "
                       "여기만 크면 더 빡빡한 쪽이 조용히 이겨, 설정값이 "
                       "실제 거동을 설명하지 못하게 된다.")
        if len(dec) == 3:
            check("velocity_smoother.max_decel[wz]", abs(float(dec[2])), float(accel_th),
                  hint="감속도 같은 값으로. 부호만 반대다.")
        if bs:
            check("behavior_server.rotational_acc_lim", bs.get("rotational_acc_lim"),
                  float(accel_th),
                  hint="BT 리커버리 Spin 의 각가속. 셋이 같아야 명령과 실제가 "
                       "한 이야기를 한다.")
        note("각가속 3곳", f"{accel_th} rad/s² "
             f"(base_control · velocity_smoother · behavior_server)",
             f"0 -> spin 상한({spin_max_wz})까지 "
             f"{float(spin_max_wz) / max(float(accel_th), 1e-9):.2f} s.")

    if vs.get("scale_velocities") is not True:
        _problems.append(("velocity_smoother.scale_velocities", str(vs.get("scale_velocities")),
                          "True",
                          "false 면 가속 램프에서 vx:wz 비가 깨져 조향각이 튄다."))
        print(f"  [{RED}불일치{RESET}] {'velocity_smoother.scale_velocities':44s} "
              f"nav2={str(vs.get('scale_velocities')):>10s}  기대=True")
    else:
        print(f"  [{GREEN}OK  {RESET}] {'velocity_smoother.scale_velocities':44s} "
              f"nav2={'True':>10s}  기대=True")

    # ---- 3) footprint / inflation ----
    print("\n" + "=" * 78)
    print("3) footprint 와 팽창반경 — 충돌평가가 성립하는가")
    print("=" * 78)
    for costmap in ("local_costmap", "global_costmap"):
        cm = _params(nav_doc, costmap, costmap)
        raw = cm.get("footprint")
        pts = yaml.safe_load(raw) if isinstance(raw, str) else raw
        if not pts:
            note(f"{costmap}.footprint", "없음")
            continue
        insc = min(min(abs(x) for x, _ in pts), min(abs(y) for _, y in pts))
        circ = max(math.hypot(x, y) for x, y in pts)
        note(f"{costmap} inscribed / circumscribed",
             f"{insc:.4f} m / {circ:.4f} m")
        infl = (cm.get("inflation_layer") or {}).get("inflation_radius")
        check_ge(f"{costmap}.inflation_radius", infl, circ,
                 hint="circumscribed 보다 작으면 costmap 기반 충돌평가와 "
                      "MPPI CostCritic 의 임계값이 성립하지 않는다.")

    # ---- 4) 조향 제한이 켜져 있는가 ----
    print("\n" + "=" * 78)
    print("4) command_manager 조향 제한 — 계획과 실행을 잇는 마지막 고리")
    print("=" * 78)
    if bc.get("steer_limit_enabled") is not True:
        _problems.append(("base_control.steer_limit_enabled",
                          str(bc.get("steer_limit_enabled")), "true",
                          "꺼두면 기구가 못 내는 곡률을 요구해도 조향각만 포화되고 "
                          "twist 는 그대로 나간다(계획≠실제)."))
        print(f"  [{RED}불일치{RESET}] {'steer_limit_enabled':44s} "
              f"     ={str(bc.get('steer_limit_enabled')):>10s}  기대=true")
    else:
        print(f"  [{GREEN}OK  {RESET}] {'steer_limit_enabled':44s} "
              f"     ={'true':>10s}  기대=true")
    min_vx = float(bc.get("steer_limit_min_vx", 0.03))
    note("steer_limit_min_vx", f"{min_vx:.3f} m/s",
         "이 아래에서는 normal 모드가 회전을 못 만들어 wz=0 으로 접힌다.")
    spin_ang = float(bc.get("auto_spin_angular_threshold", 0.35))
    spin_lin = float(bc.get("auto_spin_linear_threshold", 0.04))
    note("제자리 회전 진입 조건", f"|wz| >= {spin_ang:.2f} AND 선속도 <= {spin_lin:.2f}")
    if spin_lin < min_vx:
        _problems.append(("auto_spin_linear_threshold", f"{spin_lin:.3f}",
                          f">= steer_limit_min_vx({min_vx:.3f})",
                          "조향 클램프가 wz 를 0 으로 접는 저속 구간인데 spin 진입도 "
                          "안 되는 사각지대가 생긴다."))
        print(f"  [{RED}불일치{RESET}] {'auto_spin_linear_threshold':44s} "
              f"     ={spin_lin:>10.3f}  요구>={min_vx:.3f}")
    else:
        print(f"  [{GREEN}OK  {RESET}] {'저속 사각지대 없음 (spin_lin >= min_vx)':44s} "
              f"     ={spin_lin:>10.3f}  요구>={min_vx:.3f}")

    # ---- 5) 조향 응답 (기구가 명령을 따라올 수 있는가) ----
    print("\n" + "=" * 78)
    print("5) 조향 응답 — 명령이 기구가 낼 수 있는 속도인가")
    print("=" * 78)
    rate = bc.get("max_steer_rate_deg_s")
    check_present("base_control.max_steer_rate_deg_s", rate,
                  hint="조향 슬루 제한. 없거나 0 이면 출발 선회에서 조향 명령이 "
                       "1500 deg/s 로 튄다(|wz|<=|vx|/R_min 클램프가 걸리는 순간 "
                       "R=R_min = 정의상 최대 조향각이 되기 때문).")
    # vx=0 구간(기동 정렬 · 모드 전환 dwell)은 **정지 슬루율 S_정지**로 나눠야 한다.
    # 정지 조향은 접지면 전체를 비트는 것이라 주행 중보다 느리다 — rate 로 나누면
    # 필요시간을 과소평가한다. steer_bench.py 실측치가 base_control.yaml 에 있다.
    stop_raw = bc.get("steer_rate_stopped_deg_s")
    stop_measured = stop_raw is not None and float(stop_raw) > 0
    stop_rate = float(stop_raw) if stop_measured else (float(rate or 0.0) or 0.0)
    if stop_measured:
        note("steer_rate_stopped_deg_s (실측)", f"{stop_rate:.1f} deg/s",
             "정지 상태 조향 슬루율. 아래 두 dwell 은 vx=0 구간이라 이 값으로 나눈다.")
    else:
        note("steer_rate_stopped_deg_s", "미설정",
             f"정지 조향 슬루율 S_정지 가 없어 주행 중 값({rate} deg/s)으로 대체한다. "
             "정지 조향이 더 느리므로 아래 dwell 판정이 낙관적이다 — "
             "ros2 run alm_bringup steer_bench.py 로 실측할 것.")

    if rate is not None and float(rate) > 0:
        full = math.degrees(wis.max_steer_rad)
        t_full = full / float(rate)
        note("0° -> 최대조향 전환시간", f"{t_full:.2f} s "
             f"(vx={max_lx} 에서 주행 {max_lx * t_full:.2f} m)",
             "코너 진입 거리다. 너무 길면 코너에서 안쪽으로 파고든다.")
        if stop_measured and float(rate) > stop_rate:
            note("주행 중 슬루 명령 vs S_정지",
                 f"{float(rate):.1f} > {stop_rate:.1f} deg/s",
                 "굴러갈 때는 정지보다 빠르므로 보통 문제가 아니다. 다만 출발 직후 "
                 "(vx≈steer_limit_min_vx)에는 명령이 실제를 잠깐 앞선다 — 주행 중 "
                 "슬루율은 steering_observer 로 따로 확인할 것.")
        align = bc.get("startup_steer_align_sec")
        auto_align = 2.0 * full / stop_rate
        if align is not None and float(align) == 0.0:
            note("기동 조향 정렬 (자동)", f"{auto_align:.2f} s",
                 "전원 투입 시 조향각을 모르므로 최악 가동범위를 개루프로 기다린다.")
        elif align is not None and float(align) < 0:
            _problems.append(("startup_steer_align_sec", f"{align}", ">= 0",
                              "비활성이면 조향각을 모르는 채로 출발한다."))
            print(f"  [{RED}불일치{RESET}] {'startup_steer_align_sec':44s} "
                  f"     ={float(align):>10.2f}  요구>=0")
        elif float(align or 0) < auto_align:
            print(f"  [{YELLOW}주의{RESET}] {'기동 정렬이 전 가동범위를 덮는가':44s} "
                  f"     ={float(align or 0):>10.2f} s  필요~{auto_align:.2f} s")
            note("", "",
                 f"전 행정 {2*full:.0f}° 를 S_정지 {stop_rate:.1f} deg/s 로 펴려면 "
                 f"{auto_align:.2f} s 다. 짧으면 조향이 덜 펴진 채 출발한다.")
        else:
            print(f"  [{GREEN}OK  {RESET}] {'기동 정렬이 전 가동범위를 덮는가':44s} "
                  f"{float(align or 0):>10.2f} s >= {auto_align:.2f} s")
    else:
        _problems.append(("max_steer_rate_deg_s", str(rate), "> 0",
                          "0/미선언이면 조향 슬루 제한이 없다."))
    # 모드 전환 dwell 이 조향축 스윕을 실제로 덮는가.
    # STM32 는 모드마다 조향 자세가 고정이다: normal ±max_steer, spin CONS(9),
    # crab CONS(8). 액추에이터는 '지금 각도'에서 새 자세까지 슬루로 간다.
    dwell = bc.get("mode_switch_dwell_sec")
    if dwell is not None and stop_rate > 0:
        d = float(dwell)
        ms = float(bc.get("max_steer_deg", 30.0))
        spin_a, crab_a = 47.0, 90.0          # STM32 CONS(9), CONS(8)
        worst, worst_spin = ms + crab_a, ms + spin_a
        need, need_spin = worst / stop_rate, worst_spin / stop_rate
        note("mode_switch_dwell_sec", f"{d:.2f} s",
             f"normal(±{ms:.0f}°)/spin({spin_a:.0f}°)/crab({crab_a:.0f}°) 전환 시 "
             "조향축 스윕 시간.")
        if d < need_spin:
            print(f"  [{YELLOW}주의{RESET}] {'dwell 이 normal->spin 최악 스윕을 덮는가':44s} "
                  f"     ={d:>10.2f} s  필요~{need_spin:.2f} s")
            note("", "",
                 f"최악은 반대쪽 풀락({ms:.0f}°)에서 제로턴 자세({spin_a:.0f}°)까지 "
                 f"{worst_spin:.0f}° 를 S_정지 {stop_rate:.1f} deg/s 로 도는 것 "
                 f"= {need_spin:.2f} s. dwell 이 짧으면 조향축이 아직 도는 중에 바퀴가 "
                 "굴러 의도와 다른 궤적이 난다. ALIGN(7절)이 켜져 있으면 이 전환이 더 잦다.")
        else:
            print(f"  [{GREEN}OK  {RESET}] {'dwell 이 normal->spin 최악 스윕을 덮는가':44s} "
                  f"{d:>10.2f} s >= {need_spin:.2f} s")
        if d < need:
            note("crab 전환은 더 오래 걸림", f"필요~{need:.2f} s (현재 {d:.2f} s)",
                 "crab 은 자율주행에서 거의 안 쓰이므로 경고로만 남긴다.")
    else:
        note("mode_switch_dwell_sec", f"{dwell}",
             "normal/crab/spin 전환 시 조향축 스윕 시간.")
    if bc.get("hard_stop_on_timeout") is True:
        note("hard_stop_on_timeout", "true",
             "cmd timeout/odom stale 도 즉시 정지한다. 감속 램프를 원하면 false.")

    # ---- 6) URDF vs STM32 CONS (단일 진실 공급원 정합) ----
    print("\n" + "=" * 78)
    print("6) URDF vs STM32 CONS — 같은 로봇을 같은 숫자로 부르는가")
    print("=" * 78)
    urdf_path, urdf_src = _share("alm_description", "urdf", "alm_robot.urdf.xacro")
    props = _xacro_props(urdf_path) if urdf_path else {}
    if not props:
        note("URDF", "읽지 못함",
             "alm_description 이 없거나 xacro 상수 형식이 바뀌었다. 건너뛴다.")
    else:
        note("URDF 출처", f"[{urdf_src}] {urdf_path}")
        fx, rx = props.get("front_x"), props.get("rear_x")
        ht, wr = props.get("half_track"), props.get("wheel_radius")
        if fx is not None and rx is not None:
            check("URDF 휠베이스 (front_x - rear_x)", fx - rx, wis.B, tol=0.02,
                  hint="STM32 CONS(2) 와 다르면 선회 계산과 실제 기구가 어긋난다. "
                       "어느 쪽이 실측인지 확인 필요 (docs/TODO.md §3.95).")
        if ht is not None:
            check("URDF 윤거 (half_track * 2)", ht * 2.0, wis.half_track * 2.0,
                  tol=0.02, hint="STM32 CONS(3) 와 다르면 위와 같은 문제.")
        # footprint 를 URDF 에서 다시 유도해 nav2.yaml 값과 대조
        body_l = props.get("body_length")
        body_w = props.get("body_width")
        wheel_w = props.get("wheel_width")
        if None not in (fx, rx, ht, wr, body_l, body_w, wheel_w):
            front = fx + wr
            back = min(rx - wr, -body_l / 2.0)
            side = max(ht + wheel_w / 2.0, body_w / 2.0)
            note("URDF 유도 footprint",
                 f"x[{back:.4f}, {front:.4f}]  y[±{side:.4f}]")
            cm = _params(nav_doc, "local_costmap", "local_costmap")
            raw = cm.get("footprint")
            pts = yaml.safe_load(raw) if isinstance(raw, str) else raw
            if pts:
                nf = max(x for x, _ in pts)
                nb = min(x for x, _ in pts)
                ns = max(abs(y) for _, y in pts)
                ok = nf >= front - 1e-6 and nb <= back + 1e-6 and ns >= side - 1e-6
                mark = f"{GREEN}OK  {RESET}" if ok else f"{RED}부족{RESET}"
                print(f"  [{mark}] {'nav2 footprint 가 URDF 를 덮는가':44s} "
                      f"nav2 x[{nb:.3f}, {nf:.3f}] y[±{ns:.3f}]")
                if not ok:
                    _problems.append((
                        "costmap footprint", f"x[{nb:.3f},{nf:.3f}] y[±{ns:.3f}]",
                        f"x[{back:.3f},{front:.3f}] y[±{side:.3f}] 이상",
                        "footprint 가 실제 차체보다 작으면 충돌검사가 통과시키는 "
                        "경로에서 실제로 부딪힌다."))

    # ---- 7) ALIGN (경로 헤딩 정렬 기동) ----
    print("\n" + "=" * 78)
    print("7) ALIGN — 전역 경로가 표현 못 하는 제자리 회전을 제어단이 메우는가")
    print("=" * 78)
    if bc.get("align_enabled") is not True:
        note("align_enabled", str(bc.get("align_enabled")),
             "OFF 다. 전역 경로는 R_min 원호/직선만 담으므로, 접근 자세가 크게 "
             "어긋난 목표는 실패할 수 있다.")
    else:
        enter = float(bc.get("align_enter_deg", 0.0))
        exit_ = float(bc.get("align_exit_deg", 0.0))
        if exit_ >= enter:
            _problems.append((
                "align_exit_deg", f"{exit_}", f"< align_enter_deg({enter})",
                "히스테리시스가 없으면 진입/이탈이 매 틱 왕복한다 — "
                "예전 반경기반 라우팅이 정확히 이렇게 실패했다."))
            print(f"  [{RED}불일치{RESET}] {'align_exit_deg < align_enter_deg':44s} "
                  f"     ={exit_:>10.1f}  요구<{enter}")
        else:
            print(f"  [{GREEN}OK  {RESET}] {'히스테리시스 (exit < enter)':44s} "
                  f"{exit_:>10.1f}° < {enter:.1f}°")
        # ALIGN 의 wz 상한은 spin 상한과 같아야 한다 (command_manager 가 그렇게 넘긴다)
        note("ALIGN 회전속도 상한", f"{bc.get('auto_spin_max_angular_speed')} rad/s",
             "auto_spin_max_angular_speed 를 그대로 쓴다. spin 모드의 상한과 "
             "따로 놀 이유가 없다.")
        # 타임아웃이 dwell 보다 충분히 커야 한 번은 실제로 돈다
        dwell = float(bc.get("mode_switch_dwell_sec", 0.0) or 0.0)
        amax = float(bc.get("align_max_sec", 0.0))
        wzmax = float(bc.get("auto_spin_max_angular_speed", 0.45))
        need = math.radians(180.0) / max(wzmax, 1e-6)
        if amax < need:
            print(f"  [{YELLOW}주의{RESET}] {'align_max_sec 가 180° 회전에 부족':44s} "
                  f"     ={amax:>10.1f} s  필요~{need:.1f} s")
            note("", "", f"{wzmax} rad/s 로 180° 를 돌려면 {need:.1f} s 걸린다. "
                 "타임아웃이 그보다 짧으면 큰 각도는 절대 못 맞춘다.")
        else:
            print(f"  [{GREEN}OK  {RESET}] {'align_max_sec 가 180° 회전을 담는가':44s} "
                  f"{amax:>10.1f} s >= {need:.1f} s")
        note("모드 전환 dwell 과의 관계",
             f"dwell {dwell:.1f} s x 2 (진입+복귀) = {2*dwell:.1f} s",
             "ALIGN 1회의 고정비다. dwell 동안은 타임아웃 시계가 멈추므로 "
             "align_max_sec 를 잡아먹지는 않는다.")
        # ---- 이탈 문턱 vs 목표 자세 허용치 (2026-08-25 추가) ----
        # ##함정## ALIGN 이 손을 떼는 각도가 목표 체커 허용치보다 **크면**,
        #   그 사이 구간을 담당하는 주체가 아무도 없다:
        #     · MPPI 는 Ackermann 이라 제자리 회전을 못 낸다(|wz| <= |vx|/R_min)
        #     · ALIGN 은 진입 문턱 아래에서 다시 안 걸린다
        #   그러면 목표 근처에서 자세만 안 맞은 채로 시간을 태운다.
        gc = _params(nav_doc, "controller_server").get("general_goal_checker") or {}
        yaw_tol = gc.get("yaw_goal_tolerance")
        if yaw_tol is not None:
            yaw_tol_deg = math.degrees(float(yaw_tol))
            if exit_ > yaw_tol_deg:
                _problems.append((
                    "align_exit_deg", f"{exit_}°",
                    f"<= yaw_goal_tolerance({yaw_tol_deg:.1f}°)",
                    "ALIGN 이 목표 체커 허용치 **밖에서** 손을 뗀다. 그 사이 "
                    f"{exit_ - yaw_tol_deg:.1f}° 를 맞출 주체가 없다 — MPPI 는 "
                    "Ackermann 이라 제자리 회전을 못 낸다."))
            else:
                note("이탈 문턱 vs 목표 자세 허용치",
                     f"{exit_:.0f}° <= {yaw_tol_deg:.1f}°",
                     "ALIGN 이 목표 체커 허용치 안쪽에서 넘긴다. 마지막 몇 도를 "
                     "MPPI 에 떠넘기지 않는다.")

        note("진입 문턱 근거", f"{enter:.0f}° / {bc.get('align_enter_hold_sec')} s 지속",
             "alm_lab 실측 |경로 헤딩오차| p50 8.8° p90 39.8° p99 67.2°. "
             "30° 로 낮추면 정상 주행 틱의 35% 가 걸려 vx 가 무너진다.")

        # ---- 출발 시 헤딩오차 상한 (2026-08-25 추가) ----
        # ##함정## Hybrid-A* 는 시작 상태 theta 를 **현재 로봇 헤딩**으로 두고
        #   R_min 원호만 이어 붙인다. 즉 경로는 항상 내 헤딩에 **접해서**
        #   출발하므로, 출발 시점에 경로 헤딩오차가 낼 수 있는 최대값은
        #       |err|_max = lookahead / R_min      (호각)
        #   으로 묶인다. 진입 문턱이 그보다 크면 출발 시점에는 **수학적으로
        #   절대 안 걸린다** — 값은 멀쩡해 보이는데 기능이 죽어 있는 부류다.
        #   (lookahead 1.0 m + 문턱 60° 조합이 정확히 그 상태였다.)
        look_run = float(bc.get("align_lookahead_m") or 1.0)
        look_stp = float(bc.get("align_lookahead_m_stopped") or 0.0) or look_run
        cap_run = math.degrees(look_run / r_min)
        cap_stp = math.degrees(look_stp / r_min)
        note("출발 시 헤딩오차 상한 (lookahead / R_min)",
             f"주행 중 {cap_run:.1f}° / 정지 중 {cap_stp:.1f}°",
             f"lookahead {look_run:.2f} m / {look_stp:.2f} m, R_min {r_min:.3f} m. "
             f"경로가 내 헤딩에 접해서 출발하므로 이 값을 못 넘는다.")

        # ---- 정지 상태 진입 조건 ----
        esd = bc.get("align_enter_deg_stopped")
        ehs = bc.get("align_enter_hold_sec_stopped")
        exit_deg = float(bc.get("align_exit_deg") or 0.0)
        if esd is not None and float(esd) > 0.0:
            if float(esd) <= exit_deg:
                _problems.append((
                    "align_enter_deg_stopped", str(esd), f"> align_exit_deg({exit_deg})",
                    "정지 진입 문턱이 이탈 문턱 이하면 진입 즉시 이탈해 왕복한다."))
            elif float(esd) > cap_stp:
                need = math.radians(float(esd)) * r_min
                _problems.append((
                    "align_enter_deg_stopped", f"{esd}° (상한 {cap_stp:.1f}°)",
                    f"<= {cap_stp:.1f}° 또는 lookahead >= {need:.2f} m",
                    "출발 시점에 도달 불가능한 문턱이라 경로 헤딩만으로는 "
                    "ALIGN 이 안 걸린다. align_lookahead_m_stopped 를 올리거나 "
                    "문턱을 낮출 것."))
            else:
                note("정지 상태 진입 문턱",
                     f"{esd}° / {ehs} s 지속  (주행 중은 {bc.get('align_enter_deg')}° / "
                     f"{bc.get('align_enter_hold_sec')} s)",
                     f"정지 중 상한 {cap_stp:.1f}° 안이라 실제로 도달 가능하다. "
                     f"정지 전용 조건의 실체는 '문턱' 보다 **짧은 지속시간과 "
                     f"긴 lookahead** 다.")
            note("정지 판정 기준", f"|vx| < {bc.get('align_stopped_vx')} m/s",
                 "직전 틱에 내보낸 vx 로 판단한다 (/Odometry.twist 는 FAST-LIO 가 "
                 "안 채운다). dwell/기동정렬 구간은 진입 판정에서 제외된다 — "
                 "안 그러면 로봇이 안 굴러간 채 재진입하는 루프가 된다.")
        else:
            note("정지 상태 진입 문턱", "미설정",
                 "align_enter_deg_stopped 를 두면 출발 전에 헤딩을 맞출 수 있다.")

        # ---- 출발 전 목표 방위각 정렬 (2026-08-25 추가) ----
        # 위 lookahead 확장으로도 못 뚫는 경우가 있다. 경로가 내 헤딩에 접해서
        # 출발한다는 사실 자체는 안 바뀌기 때문이다(벽을 보고 선 채 뒤쪽 목표).
        # 그때는 경로가 아니라 **목표까지의 직선 방위각**을 봐야 한다.
        if bc.get("align_goal_bearing_enabled") is True:
            bdeg = float(bc.get("align_goal_bearing_deg") or 0.0)
            if bdeg <= exit_deg:
                _problems.append((
                    "align_goal_bearing_deg", str(bdeg), f"> align_exit_deg({exit_deg})",
                    "방위각 진입 문턱이 이탈 문턱 이하면 진입 즉시 이탈한다."))
            else:
                note("출발 전 목표 방위각 정렬",
                     f"{bdeg}° 초과 + 목표까지 "
                     f"{bc.get('align_goal_bearing_min_dist_m')} m 이상, 목표당 1회",
                     "'벽을 보고 선 채 뒤쪽 목표' 를 뚫는 유일한 경로다. "
                     "아직 안 굴러간 정지 상태에서만 쓴다 — 주행 중에는 목표 "
                     "방위각이 경로와 다른 게 정상이라 쫓으면 벽으로 간다.")
        else:
            _problems.append((
                "align_goal_bearing_enabled", str(bc.get("align_goal_bearing_enabled")),
                "true",
                "OFF 면 '벽을 보고 선 채 뒤쪽 목표' 에서 R_min 으로 크게 감아 도는 "
                "경로가 그대로 나간다. 경로 헤딩만으로는 못 고치는 상황이다."))

        # ---- 정지 중 재래치 ----
        if bc.get("align_relatch_stopped") is True:
            per = max(cap_stp, float(esd or 0.0)) - exit_deg
            note("정지 중 목표 재래치", "ON",
                 f"OFF 면 한 기동이 도는 각도가 최대 {per:.0f}° 로 묶인다 "
                 f"(래치각 - 이탈각). 왕복 1회 비용이 dwell {2*dwell:.0f} s 라 "
                 f"큰 각도를 돌리면 시간예산이 통째로 날아간다.")
        else:
            note("정지 중 목표 재래치", "OFF",
                 "한 기동이 도는 각도가 '래치각 - 이탈각' 으로 묶인다.")

    # ---- 결과 ----
    print("\n" + "=" * 78)
    if not _problems:
        print(f"{GREEN}전부 일치합니다.{RESET}")
        print("=" * 78 + "\n")
        return 0
    print(f"{RED}어긋난 항목 {len(_problems)}개{RESET}")
    print("=" * 78)
    for label, actual, expected, hint in _problems:
        print(f"\n  · {label}\n      현재={actual}   기대={expected}")
        if hint:
            print(f"      {DIM}{hint}{RESET}")
    print("\n  어느 쪽이 맞는지는 사람이 정해야 합니다. 기구 상수를 바꿨다면 "
          "nav2.yaml 을,\n  Nav2 를 의도적으로 보수적으로 두는 중이면 그대로 두세요.\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
