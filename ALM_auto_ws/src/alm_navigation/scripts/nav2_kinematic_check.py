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
    note("normal 모드 실제 상한", f"{wz_normal:.4f} rad/s @ vx={max_lx}",
         f"MPPI wz_max({fp.get('wz_max')})와의 차이는 조향 클램프 + spin 라우팅이 흡수한다.")

    vs = _params(nav_doc, "velocity_smoother")
    mv, mnv = vs.get("max_velocity") or [], vs.get("min_velocity") or []
    if len(mv) == 3 and len(mnv) == 3:
        check("velocity_smoother.max_velocity[vx]", mv[0], max_lx)
        check("velocity_smoother.min_velocity[vx]", mnv[0], min_lx)
        check("velocity_smoother.max_velocity[wz]", mv[2], spin_max_wz)
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
