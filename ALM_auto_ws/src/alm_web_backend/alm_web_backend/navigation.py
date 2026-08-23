"""자율주행 목표 전송 전 점검과 입력 검증.

localization.py 와 같은 자리에 있는 파일이다 — **명령을 보내기 전에 거부할 수
있는 것은 보내기 전에 거부한다.** 측위가 조용히 실패하는 것이 그 파일의 이유였다면,
여기는 목표가 조용히 실패하는 것이 이유다.

Nav2 는 전제가 안 맞아도 목표를 **받아준다.** map->odom TF 가 없으면
ComputePathToPose 가 실패하고 BT 가 리커버리를 몇 바퀴 돈 뒤 abort 하는데,
그 사이 화면은 'RUNNING' 이고 로그는 리커버리 이름만 지나간다. 조작자는 로봇이
길을 찾는 중이라고 믿는다. 실제로는 자기 위치를 모르는 것이다.

그래서 여기서 먼저 묻는다:
    1) Nav2 액션 서버가 떠 있나          — 없으면 목표는 그냥 사라진다
    2) map->odom TF 가 있나 (= 측위 수렴) — 없으면 계획 자체가 불가능하다

좌표 검증도 여기 있다. 브라우저가 SVG 픽셀을 미터로 바꿔 보내므로, 변환이
틀어지면 x=41000 같은 값이 온다. Nav2 는 그걸 받아 코스트맵 밖이라며 실패하는데,
그때 화면에 뜨는 것은 'planner failed' 뿐이라 원인이 좌표라는 것을 알 수 없다.
"""

import math

# 한 미션의 목표 개수 상한. FollowWaypoints 자체에는 제한이 없지만, 브라우저가
# 실수로 클릭을 쏟아부었을 때 로봇이 몇 시간짜리 미션을 받는 것을 막는다.
MAX_WAYPOINTS = 50

# 좌표 상한 [m]. 실내 맵에서 이 밖은 전부 입력 오류다 (변환 버그 또는 오클릭).
# 맵 크기로 자동 판정하지 않는 이유: 맵을 바꾸는 중이거나 map_server 가 아직
# 안 떴을 때 '검증이 조용히 꺼지는' 상태가 생기기 때문이다.
COORD_LIMIT_M = 1000.0

GOAL_FRAME = "map"


class PreflightError(Exception):
    """전송 전 점검 실패. 사용자에게 그대로 보여줄 문장을 담는다."""


def check_stack(*, action_ready, tf_ready, slot_running, external_nodes):
    """목표를 보낼 수 있는 상태인지. 못 보내면 PreflightError.

    slot_running / external_nodes 는 거부 사유를 **구체적으로** 쓰기 위해서만
    받는다. 판정 자체는 action_ready 와 tf_ready 가 한다 — 웹이 띄웠는지
    CLI 로 띄웠는지는 목표를 받을 수 있는가와 무관하기 때문이다.
    """
    if not action_ready:
        if slot_running:
            # 슬롯은 도는데 액션이 없다 = 아직 기동 중이거나 nav2 가 죽었다.
            raise PreflightError(
                "자율주행 스택이 기동 중입니다 — Nav2 액션 서버가 아직 광고되지 "
                "않았습니다. 몇 초 뒤 다시 시도하세요. (계속 이 상태면 자율주행 "
                "로그를 확인하세요 — 플래너 설정 오류면 여기서 멈춥니다.)")
        raise PreflightError(
            "Nav2 가 떠 있지 않습니다. '자율주행 시작' 으로 스택을 먼저 기동하세요."
            + (f" (측위 노드 {', '.join(external_nodes)} 는 떠 있습니다 — 측위만 "
               f"기동된 상태입니다.)" if external_nodes else ""))

    if not tf_ready:
        raise PreflightError(
            "아직 초기위치가 잡히지 않았습니다 (map→odom TF 없음). "
            "FPFH+TEASER++ 정합이 끝나야 경로를 계획할 수 있습니다 — "
            "로봇을 정지시킨 채로 측위가 수렴하기를 기다리세요.")


def parse_point(raw, *, index=None):
    """{x, y, yaw_deg} 한 개를 검증해 정규화한다."""
    where = f"{index + 1}번 목표" if index is not None else "목표"
    if not isinstance(raw, dict):
        raise PreflightError(f"{where} 형식이 올바르지 않습니다 (객체여야 합니다).")

    out = {}
    for key in ("x", "y"):
        try:
            value = float(raw.get(key))
        except (TypeError, ValueError):
            raise PreflightError(f"{where}의 {key} 가 숫자가 아닙니다.") from None
        if not math.isfinite(value):
            raise PreflightError(f"{where}의 {key} 가 유한한 값이 아닙니다.")
        if abs(value) > COORD_LIMIT_M:
            raise PreflightError(
                f"{where}의 {key}={value:.1f} m 는 범위를 벗어납니다 "
                f"(±{COORD_LIMIT_M:.0f} m). 지도 좌표 변환을 확인하세요.")
        out[key] = value

    # yaw 는 선택이다. 안 주면 0°. 이 플랫폼에서 자세 지정 목표는 아직
    # 플래너가 잘 못 푸는 영역이라(docs/control_pipeline.md §12.5.2) 기본값을
    # 강제하지 않고 받은 대로 넘긴다.
    try:
        yaw_deg = float(raw.get("yaw_deg", raw.get("yaw", 0.0)) or 0.0)
    except (TypeError, ValueError):
        raise PreflightError(f"{where}의 yaw 가 숫자가 아닙니다.") from None
    if not math.isfinite(yaw_deg):
        raise PreflightError(f"{where}의 yaw 가 유한한 값이 아닙니다.")
    out["yaw_deg"] = ((yaw_deg + 180.0) % 360.0) - 180.0

    label = raw.get("label", "")
    out["label"] = str(label)[:60] if label else ""
    return out


def parse_points(raw, *, max_points=MAX_WAYPOINTS):
    """웨이포인트 목록을 검증해 정규화한다."""
    if not isinstance(raw, list) or not raw:
        raise PreflightError("웨이포인트가 없습니다. 지도에서 목표를 하나 이상 추가하세요.")
    if len(raw) > max_points:
        raise PreflightError(
            f"웨이포인트가 너무 많습니다 ({len(raw)}개, 최대 {max_points}개).")
    return [parse_point(entry, index=i) for i, entry in enumerate(raw)]


def yaw_to_quaternion(yaw_deg):
    """평면 회전만. (z, w) 를 돌려준다 — x, y 는 항상 0 이다."""
    half = math.radians(yaw_deg) / 2.0
    return math.sin(half), math.cos(half)


def describe(points):
    """미션 한 줄 요약. 로그와 음성 문구에 함께 쓴다."""
    if len(points) == 1:
        point = points[0]
        return f"x={point['x']:.2f} y={point['y']:.2f} yaw={point['yaw_deg']:.0f}deg"
    return f"웨이포인트 {len(points)}개"


def path_length(points, start=None):
    """직선 거리 합 [m]. Nav2 의 실제 경로장이 아니라 **하한 추정**이다.

    Hybrid-A* 는 R_min=1.643 m 원호로만 이으므로 실제 경로는 항상 이보다 길다.
    화면에 '남은 거리'로 쓰지 않는다 — 그건 NavigateToPose 피드백의
    distance_remaining 이 준다. 여기 값은 미션을 받을 때 규모를 가늠하는 용도다.
    """
    total = 0.0
    previous = start
    for point in points:
        if previous is not None:
            total += math.hypot(point["x"] - previous["x"], point["y"] - previous["y"])
        previous = point
    return round(total, 2)
