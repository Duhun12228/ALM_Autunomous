#!/usr/bin/env python3
"""alm_web_backend 자율주행 엔드포인트 시나리오 시험.

하드웨어도 Nav2 도 없이 /api/navigation/* 전 경로를 돌린다. Nav2 자리에는
alm_bringup/scripts/fake_nav2.py 가 들어간다 — 액션 서버 둘과 map->odom TF 를
흉내내므로, 백엔드 입장에서는 진짜와 구분되지 않는다.

    # 터미널 1
    export ALM_WEB_TOKEN=$(openssl rand -hex 16); echo "$ALM_WEB_TOKEN" > /tmp/alm.tok
    ros2 run alm_web_backend web_backend.py
    # 터미널 2
    ros2 run alm_bringup fake_nav2.py --step 0.6
    # 터미널 3
    python3 nav_api_test.py /tmp/alm.tok

    # 실패 경로도 본다 (abort / 일부 목표 건너뜀 / 측위 미수렴)
    ros2 run alm_bringup fake_nav2.py --fail
    ros2 run alm_bringup fake_nav2.py --no-tf
"""
import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://localhost:8081"
TOKEN = open(sys.argv[1] if len(sys.argv) > 1 else "tok").read().strip()
SESSION = ""
fails = []


def call(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method)
    req.add_header("Authorization", f"Bearer {TOKEN}")
    req.add_header("Content-Type", "application/json")
    if SESSION:
        req.add_header("X-ALM-Session", SESSION)
    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def check(label, ok, detail=""):
    print(f"  [{'OK ' if ok else 'FAIL'}] {label}{(' — ' + str(detail)) if detail else ''}")
    if not ok:
        fails.append(label)


def mission():
    return call("GET", "/api/navigation")[1]["mission"]


def wait_state(target, timeout=25.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        state = mission()["state"]
        if state in target:
            return state
        time.sleep(0.2)
    return mission()["state"]


status, body = call("POST", "/api/session/acquire", {"label": "navtest"})
SESSION = body["session_id"]
print(f"\n세션 확보: {SESSION}\n")

print("① 사전 점검")
status, body = call("GET", "/api/navigation")
check("액션 서버 감지", body["mission"]["action_ready"], body["mission"]["action_ready"])
check("map->odom TF 감지", body["mission"]["tf_ready"], body["mission"]["tf_ready"])

print("② 입력 검증")
status, body = call("POST", "/api/navigation/goal", {"points": [{"x": 99999, "y": 0}]})
check("좌표 범위 초과 거부(400)", status == 400, f"{status} {body.get('error','')[:50]}")
status, body = call("POST", "/api/navigation/goal", {"points": []})
check("빈 목표 거부(400)", status == 400, status)
status, body = call("POST", "/api/navigation/goal",
                    {"points": [{"x": 1, "y": 1}] * 60})
check("개수 초과 거부(400)", status == 400, status)

print("③ 단일 목표 — NavigateToPose")
status, body = call("POST", "/api/navigation/goal", {"points": [{"x": 2.0, "y": 1.0, "yaw_deg": 90}]})
check("목표 수락(200)", status == 200, f"{status} {body.get('message', body.get('error'))}")
check("kind=pose", body["mission"]["kind"] == "pose", body["mission"]["kind"])
time.sleep(2.5)
live = mission()
check("피드백으로 남은거리 갱신", live["distance_remaining_m"] is not None,
      f"distance={live['distance_remaining_m']} eta={live['eta_sec']}")
check("중복 목표 거부", call("POST", "/api/navigation/goal",
                        {"points": [{"x": 0, "y": 0}]})[0] == 409)
check("도달로 종료", wait_state({"succeeded"}) == "succeeded", mission()["message"])

print("④ 웨이포인트 — 일시정지 / 재개")
points = [{"x": 1.0, "y": 0.0}, {"x": 2.0, "y": 0.0}, {"x": 3.0, "y": 0.0}]
status, body = call("POST", "/api/navigation/goal", {"points": points})
check("미션 수락", status == 200, body.get("message", body.get("error")))
check("kind=waypoints", body["mission"]["kind"] == "waypoints", body["mission"]["kind"])
check("직선 추정거리", body["mission"]["distance_estimate_m"] == 2.0,
      body["mission"]["distance_estimate_m"])
time.sleep(1.3)
before = mission()
status, body = call("POST", "/api/navigation/pause", {})
check("일시정지(200)", status == 200, body.get("message", body.get("error")))
paused = wait_state({"paused"}, 6)
check("state=paused", paused == "paused", paused)
remaining = mission()["remaining"]
check("남은 목표 보존", 0 < len(remaining) <= 3,
      f"index={mission()['index']} 남음={len(remaining)}")
status, body = call("POST", "/api/navigation/resume", {})
check("재개(200)", status == 200, body.get("message", body.get("error")))
check("재개 후 전체 목록 유지", mission()["total"] == 3, mission()["total"])
check("미션 완료", wait_state({"succeeded", "failed"}, 25) == "succeeded", mission()["message"])

print("⑤ 중단")
call("POST", "/api/navigation/goal", {"points": [{"x": 5.0, "y": 5.0}]})
time.sleep(1.0)
status, body = call("POST", "/api/navigation/cancel", {})
check("중단(200)", status == 200, body.get("message", body.get("error")))
state = wait_state({"canceled"}, 6)
check("state=canceled", state == "canceled", state)
check("재개 거부(일시정지 아님)", call("POST", "/api/navigation/resume", {})[0] == 409)

print(f"\n{'전부 통과' if not fails else '실패: ' + ', '.join(fails)}")
sys.exit(1 if fails else 0)
