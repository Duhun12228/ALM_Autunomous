builtin_interfaces/Time stamp

# Jetson 온보드 리소스 실측치. jetson_stats_publisher 가 sysfs 에서 직접 읽는다
# (tegrastats/jtop 미사용 — 서브프로세스 없이 1 Hz 로 갱신).
# 값을 못 읽은 항목은 NaN(float) / 0(uint) 으로 채운다.

float32 cpu_percent          # 전체 CPU 사용률 [%]  (/proc/stat)
float32[] cpu_core_percent   # 코어별 사용률 [%]
float32 gpu_percent          # GPU 사용률 [%]       (/sys/devices/platform/gpu.0/load, 천분율/10)

float32 ram_used_gb          # 사용 중 RAM [GB]     (/proc/meminfo MemTotal-MemAvailable)
float32 ram_total_gb         # 전체 RAM [GB]
float32 swap_used_gb
float32 swap_total_gb

# 온도 [°C]. /sys/devices/virtual/thermal/thermal_zone*/ 의 type 으로 매칭.
float32 temp_cpu             # cpu-thermal
float32 temp_gpu             # gpu-thermal
float32 temp_soc             # soc0/1/2-thermal 중 최대
float32 temp_tj              # tj-thermal (정션, 스로틀 기준)

# 전력. ina3221 hwmon 의 VDD_IN 레일 = 보드 총 입력.
float32 power_w              # 현재 [W]
float32 power_avg_w          # 최근 평균 [W]

# 네트워크 처리량 [Mbps]. 지정 인터페이스의 /sys/class/net/<if>/statistics 차분.
string net_interface
float32 net_tx_mbps
float32 net_rx_mbps

uint32 uptime_sec
