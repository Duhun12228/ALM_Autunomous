#!/usr/bin/env python3
"""alm_web_backend 진입점 — WebUI 명령 어댑터.

    ros2 run alm_web_backend web_backend.py
    ALM_WEB_TOKEN=$(openssl rand -hex 16) ros2 run alm_web_backend web_backend.py

읽기(텔레메트리)는 foxglove_bridge:8765 가, 쓰기(명령)는 이 프로세스:8081 이
맡는다. foxglove 쪽 차단 설정은 이 패키지와 무관하게 그대로 유지된다 —
브라우저가 ROS 토픽에 직접 publish 하는 경로는 여전히 0개다.

스레드 배치:
    메인      rclpy MultiThreadedExecutor.spin()
    데몬×N    ThreadingHTTPServer 요청 핸들러 → RosInterface 메서드 호출
"""

import argparse
import os
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from alm_web_backend.api import Api
from alm_web_backend.http_server import Backend, Router, load_token, serve
from alm_web_backend.jobs import JobManager
from alm_web_backend.processes import ProcessManager
from alm_web_backend.ros_iface import RosInterface
from alm_web_backend.session import SessionLock

DEFAULT_PORT = 8081


def import_map_layout():
    """alm_navigation 의 맵 레이아웃 헬퍼를 빌려 쓴다.

    launch 파일들과 같은 방식으로 import 한다 (share/alm_navigation/launch).
    같은 규약을 두 번 구현하면 언젠가 갈라지므로 복사하지 않는다.
    """
    share = get_package_share_directory("alm_navigation")
    launch_dir = os.path.join(share, "launch")
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    import map_layout                                   # noqa: PLC0415
    return map_layout, share


def resolve_executables():
    """subprocess 로 부를 실행 파일의 절대경로.

    `ros2 run <pkg> <exe>` 를 쓰지 않는 이유: 중간에 파이썬 프로세스가 하나 더
    끼어 신호 전달과 종료 코드가 한 겹 흐려진다. 설치 경로를 직접 부른다.
    """
    paths = {}
    for key, package, name in (
        ("pcd2pgm", "alm_navigation", "pcd2pgm.py"),
        ("fpfh_map_builder", "icp_relocalization", "fpfh_map_builder"),
    ):
        try:
            prefix = get_package_prefix(package)
        except Exception:                               # noqa: BLE001
            paths[key] = ""
            continue
        paths[key] = os.path.join(prefix, "lib", package, name)
    return paths


def parse_args(argv):
    parser = argparse.ArgumentParser(description="ALM WebUI 명령 백엔드")
    parser.add_argument("--host", default=os.environ.get("ALM_WEB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int,
                        default=int(os.environ.get("ALM_WEB_PORT", DEFAULT_PORT)))
    parser.add_argument("--token", default=None,
                        help="생략하면 ALM_WEB_TOKEN 환경변수를 쓴다")
    parser.add_argument("--allow-no-auth", action="store_true",
                        help="인증 없이 기동 (격리된 개발 환경에서만)")
    parser.add_argument("--session-ttl", type=float, default=15.0,
                        help="웹 제어권 리스 만료(초). 하트비트가 이만큼 끊기면 자동 해제")
    parser.add_argument("--cors-origin", action="append", default=None,
                        help="허용할 Origin. 반복 지정 가능. 기본은 * (토큰이 실질 관문)")
    parser.add_argument("--log-dir", default=os.path.expanduser("~/.ros/alm_web_backend"))
    return parser.parse_args(argv)


def main():
    argv = remove_ros_args(args=sys.argv)[1:]
    args = parse_args(argv)

    # 기동 거부는 rclpy.init 전에 — ROS 노드를 띄워놓고 죽으면 지저분하다.
    token = load_token(args.token, args.allow_no_auth)

    map_layout, nav_share = import_map_layout()
    maps_root = map_layout.maps_root(nav_share)
    fastlio_config = os.path.join(nav_share, "config", "fastlio_mid360.yaml")

    rclpy.init()
    node = RosInterface()
    logger = node.get_logger()

    if not os.path.isfile(fastlio_config):
        logger.error(f"fastlio 설정을 찾을 수 없습니다: {fastlio_config}")

    executables = resolve_executables()
    for key, path in executables.items():
        if not path or not os.path.isfile(path):
            logger.warn(f"{key} 실행 파일 없음: {path or '(패키지 미설치)'} — "
                        f"해당 작업 요청은 500 으로 거부됩니다")

    session = SessionLock(ttl_sec=args.session_ttl)
    processes = ProcessManager(log_dir=args.log_dir, logger=logger)
    jobs = JobManager(logger=logger)

    router = Router()
    api = Api(
        ros=node,
        session=session,
        processes=processes,
        jobs=jobs,
        maps_root=maps_root,
        map_layout=map_layout,
        fastlio_config=fastlio_config,
        exe_paths=executables,
        logger=logger,
    )
    api.register(router)

    backend = Backend(
        token=token,
        session=session,
        router=router,
        logger=logger,
        cors_origins=args.cors_origin or ["*"],
    )

    try:
        httpd = serve(backend, args.host, args.port)
    except OSError as error:
        logger.error(f"{args.host}:{args.port} 바인딩 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1) from error

    logger.info(
        f"명령 백엔드 http://{args.host}:{args.port} "
        f"({'토큰 인증' if token else '⚠ 인증 없음'}, "
        f"제어권 TTL {args.session_ttl:.0f}s)")
    logger.info(f"maps_root={maps_root}")
    if not token:
        logger.warn("--allow-no-auth 로 기동했습니다. 같은 네트워크의 누구나 "
                    "로봇에 명령을 보낼 수 있습니다.")

    # 프로세스가 혼자 죽는 것을 감시한다. /api/health 는 브라우저가 열려 있을
    # 때만 폴링되므로, 실차에서 화면 없이 돌 때는 아무도 안 본다.
    node.create_timer(2.0, api.check_processes)
    # 같은 이유로 제어권 만료도 깨워줘야 한다. 리스 정리는 원래 누가 API 를
    # 부를 때 곁다리로만 일어나는데, 락을 쥔 브라우저가 크래시하고 그게
    # 유일한 접속자였다면 부를 사람이 없다. TTL 15초에 2초 폴링이면 늦어야
    # 2초 뒤에는 알린다.
    node.create_timer(2.0, session.poll)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("종료 중 — 실행 중인 launch 프로세스를 정리합니다")
        httpd.shutdown()
        processes.stop_all()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
