# Log and broadcast when a multi-agent conflict or deadlock is detected & resolved
builtin_interfaces/Time stamp
string conflict_id
string[] involved_robots

# Spatial point of the intersection / conflict zone
geometry_msgs/Point conflict_point

# Resolution strategy applied:
# "EDGE_AI_ORCA_VELOCITY_ADJUST" : Smooth speed adjustment (No stopping)
# "EDGE_AI_ALCOVE_YIELD"        : Low-priority robot pulls into side alcove
# "STOP_AND_WAIT"               : Baseline legacy behavior (Full stop)
# "REROUTE_DYNAMIC"             : Aisle blocked, dynamic recalculation
string resolution_action

string yielding_robot_id
float32 delay_incurred_sec
bool is_deadlock_resolved
