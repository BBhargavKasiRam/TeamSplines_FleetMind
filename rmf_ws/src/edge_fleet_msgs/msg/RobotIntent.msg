# Decentralized P2P Robot Intent Broadcast (Published onboard by each AMR at 10Hz)
builtin_interfaces/Time stamp
string robot_id

# Real-time state
geometry_msgs/Pose2D current_pose
geometry_msgs/Twist current_velocity
float32 battery_soc              # Battery % (0.0 - 100.0)

# Intent horizon: sequence of planned waypoints over the next T seconds
geometry_msgs/Pose2D[] predicted_path

# Mission status
string current_task_id
uint8 priority_level             # 1 (Normal) to 10 (Critical/Urgent)
bool is_yielding                 # True if robot is actively yielding at an alcove/intersection
bool is_blocked                  # True if robot encountered an obstacle/blocked aisle
