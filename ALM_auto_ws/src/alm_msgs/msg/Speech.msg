# voice_announcer 에 보내는 발화 요청 (/alm/say).
#
# std_msgs/String 을 안 쓰는 이유: E-STOP 안내가 "two D map ready" 뒤에서
# 순서를 기다리면 안 된다. 우선순위와 선점이 타입에 있어야 한다.

# 영어 짧은 문장. 2초 안에 끝나야 다음 사건을 놓치지 않는다.
# 약어는 띄어쓴다 — "SLAM" 이 아니라 "S L A M" (붙이면 합성기가 뭉갠다).
string text

uint8 PRIORITY_INFO   = 0    # 배경 정보. 밀리면 버려도 된다
uint8 PRIORITY_NORMAL = 1    # 명령 결과. 조작자가 기다리는 답
uint8 PRIORITY_SAFETY = 2    # 안전 사건. 절대 버리지 않는다
uint8 priority

# 같은 key 가 큐에 이미 있으면 갈아끼운다. 워치독처럼 반복되는 안내가
# 큐를 채워 다른 발화를 밀어내는 것을 막는다. 비우면 중복 억제 안 함.
string key

# 재생 중인 것을 끊고 즉시 말한다. 안전 사건 전용.
bool interrupt
