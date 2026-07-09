builtin_interfaces/Time stamp
uint32 sequence

# Feedback reported by the MCU to Jetson.
geometry_msgs/Twist measured_velocity
geometry_msgs/Pose2D odom_pose

# 2-axle steering feedback (조향 드라이버 2개: 앞축/뒤축). Order: [front, rear].
# 축당 두 바퀴는 타이로드로 함께 조향됨. (바퀴 독립조향으로 바뀌면 길이 4로 확장)
float32[2] steer_angle    # 조향각 [rad]
# 4-wheel independent drive feedback. Order: [front_left, front_right, rear_left, rear_right].
float32[4] wheel_speed    # 구동 바퀴 각속도 [rad/s]

float32 battery_voltage
float32 battery_current

bool motors_enabled
bool emergency_stop
bool command_timeout
bool fault
uint32 fault_code
string fault_text

