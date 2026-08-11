# maps/ 디렉터리의 현재 상태. map_manager 가 발행한다.
#
# QoS 는 TRANSIENT_LOCAL/depth 1 — 늦게 접속한 브라우저도 즉시 최신 목록을 받는다.
# 내용이 바뀔 때만 발행하고, 그 외에는 하트비트 주기로만 다시 낸다.
#
# 읽기 전용이다. 맵 생성·전환·삭제 경로는 여기에 없다.

builtin_interfaces/Time stamp

string root                  # maps/ 절대 경로
string active_map            # active.yaml 이 가리키는 맵 이름. 없으면 빈 문자열

# 지문 대조(FNV-1a) 를 실제로 수행했는가. false 면 stale 판정은
# 점 개수 + mtime 비교에만 근거한다 — UI 는 '미확인'을 미확인이라고 표시할 것.
bool fingerprint_verified

alm_msgs/MapEntry[] maps
