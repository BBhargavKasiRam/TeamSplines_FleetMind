# RobotState.msg
# Real-time state of an autonomous warehouse robot

string STATUS_IDLE=IDLE
string STATUS_MOVING=MOVING
string STATUS_PICKING=PICKING
string STATUS_DROPPING=DROPPING
string STATUS_YIELDING=YIELDING
string STATUS_WAITING=WAITING
string STATUS_CHARGING=CHARGING
string STATUS_DEADLOCKED=DEADLOCKED

string robot_id
string robot_name
string robot_namespace
string status
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity
float32 battery_percentage
string current_task_id
string carried_package_id
bool is_carrying_package
string assigned_color
