"""alm_web_backend — WebUI 명령 어댑터.

경로가 둘로 나뉘어 있다는 점이 이 패키지 설계의 전부다.

    읽기: 브라우저 ──WS:8765──▶ foxglove_bridge   (구독 전용, 쓰기 구조적 차단)
    쓰기: 브라우저 ──HTTP:8081─▶ alm_web_backend  (이 패키지)

foxglove_bridge 를 열어서 브라우저가 토픽에 직접 publish 하게 만들면,
cmd_arbiter 의 동작권과 command_manager 의 안전 게이팅이 통째로 우회된다.
그래서 쓰기는 전부 이쪽으로 돌린다.

이 패키지는 **얇은 어댑터**다. 속도 제한·명령 타임아웃·fault 정지 같은 안전
판단은 command_manager 한 곳에만 있어야 한다. 여기에 같은 판단을 또 넣으면
두 곳이 서로 다른 결론을 내는 날이 온다.
"""
