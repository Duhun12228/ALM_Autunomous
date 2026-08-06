builtin_interfaces/Time stamp
uint32 sequence

# High-level base command sent from Jetson to the MCU.
geometry_msgs/Twist cmd_vel
string drive_mode

bool enable_motors
bool emergency_stop

# --- STM32 4WIS 인터페이스 (uart_protocol.md v2, 실제 전송되는 값) ---
# 위 cmd_vel/drive_mode 를 command_manager 가 변환한 결과입니다.
# UART 프레임에는 아래 3개 + flags 만 실립니다. cmd_vel 은 디버그/로깅용.
float32 steer_deg     # 조향 명령 [deg]. 음수 = 좌회전 (STM32 내부 부호반전 반영)
float32 speed_rpm     # 구동 명령 [rpm]. 일반 주행은 '내측 전륜' 기준
uint8 mode_id         # 0=정지 1=일반 2=자가추종(미구현) 3=크랩 4=제로턴
