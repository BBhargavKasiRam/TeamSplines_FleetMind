# Real-time Head-to-Head Race Telemetry (Edge AI vs Traditional Stop-and-Wait)
builtin_interfaces/Time stamp

# Overall race state: "PRE_START", "RUNNING", "EDGE_WON", "ALL_FINISHED"
string race_state

# Mission targets
uint32 target_laps

# Edge-AI Fleet Telemetry
float32 edge_elapsed_sec
uint32 edge_laps_completed
bool edge_finished
float32 edge_finish_time_sec
uint32 edge_collision_count

# Traditional Stop-and-Wait Fleet Telemetry
float32 trad_elapsed_sec
uint32 trad_laps_completed
bool trad_finished
float32 trad_finish_time_sec
float32 trad_cumulative_idle_sec
uint32 trad_collision_count

# Comparative Performance
float32 time_saved_percent
string leader_status
