"""ROS 로거 호출 헬퍼.

⚠ 왜 이런 게 필요한가 — rclpy 의 함정 하나 때문이다.

`RcutilsLogger.log()` 는 **호출 지점(파일+줄번호)별로 severity 를 캐시**한다.
그래서 이렇게 쓰면 안 된다:

    def _log(self, level, text):
        getattr(self.logger, level)(text)      # ← 모든 severity 가 이 한 줄을 지난다

첫 호출이 INFO 였다면 그 줄은 INFO 로 굳고, 나중에 WARN 으로 부르는 순간
`ValueError: Logger severity cannot be changed between calls.` 가 난다.
HTTP 핸들러 안에서 이게 터지면 응답을 못 쓰고 연결이 끊겨, 클라이언트는
**빈 응답**을 받는다 (상태 코드조차 없다).

그래서 severity 마다 호출 줄을 따로 둔다.
"""


def log(logger, level, text):
    if logger is None:
        return
    try:
        if level == "debug":
            logger.debug(text)
        elif level == "warn":
            logger.warn(text)
        elif level == "error":
            logger.error(text)
        else:
            logger.info(text)
    except Exception:                       # noqa: BLE001
        # 로깅이 요청 처리를 깨뜨리면 안 된다. 위 함정으로 예외가 나면 응답을
        # 못 쓰고 연결이 끊겨, 클라이언트는 상태 코드도 없는 빈 응답을 받는다.
        pass
