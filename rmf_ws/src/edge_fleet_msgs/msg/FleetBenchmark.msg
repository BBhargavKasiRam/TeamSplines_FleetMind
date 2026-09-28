# Real-time quantitative benchmark telemetry comparing Edge AI vs Baseline Stop-and-Wait
builtin_interfaces/Time stamp

# Active execution mode: "EDGE_AI" or "STOP_AND_WAIT"
string active_mode

# Time metrics
float32 total_elapsed_time_sec
float32 cumulative_delay_time_sec
float32 stop_and_wait_baseline_time_sec

# Quantitative Improvement (Target: >= 20.0%)
float32 efficiency_improvement_percent

# Safety & Conflict metrics
uint32 total_conflicts_detected
uint32 dynamic_resolutions_count
uint32 deadlocks_prevented_count
uint32 collision_count             # Must strictly remain 0

# Fleet health
uint32 active_robot_count
float32 average_battery_soc
