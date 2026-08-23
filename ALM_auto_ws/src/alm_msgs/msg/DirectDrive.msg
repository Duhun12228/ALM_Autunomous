# 액추에이터 직접 명령 — twist 를 거치지 않는다.
#
# 왜 이런 것이 필요한가: twist -> rpm/조향각 변환은 base_control.yaml 의
# ##CONFIRM## 상수 8개(wheel_radius_m, gear_ratio, max_steer_deg, max_rpm,
# straight_angle_deg, crab/spin 부호, ...)에 통째로 의존한다. 그 값들이 아직
# 실차로 확정되지 않았으므로, twist 로는 "0.2 m/s 를 요청했는데 실제로 몇 m/s 가
# 나가는지" 를 아무도 모른다. 반대로 rpm 과 각도는 STM32 가 받는 단위 그대로라
# 변환이 없다 — **상수를 확정하기 위한 측정 자체가 이 경로로만 가능하다.**
#
# 안전 계층은 우회하지 않는다. command_manager 가 이 명령도 받아서
# E-STOP 래치 · 명령 타임아웃 · 조향 슬루 제한 · 모드 전환 dwell 을 그대로
# 적용한 뒤 /mcu/command 를 낸다. 바뀌는 것은 '무엇을 명령의 원본으로 보는가'
# 뿐이고, /mcu/command 의 발행자는 여전히 command_manager 하나다.

builtin_interfaces/Time stamp

float32 speed_rpm     # 구동 명령 [rpm]. 일반 주행은 '내측 전륜' 기준 (McuCommand 와 같은 규약)
float32 steer_deg     # 조향 명령 [deg]. 음수 = 좌회전. mode 1(일반) 에서만 쓰인다
uint8 mode_id         # 0=정지 1=일반 3=크랩 4=제로턴 (McuCommand.mode_id 와 같은 값)
