builtin_interfaces/Time stamp
uint32 sequence

# Feedback reported by the MCU to Jetson.
geometry_msgs/Twist measured_velocity
geometry_msgs/Pose2D odom_pose

float32 battery_voltage
float32 battery_current

bool motors_enabled
bool emergency_stop
bool command_timeout
bool fault
uint32 fault_code
string fault_text

