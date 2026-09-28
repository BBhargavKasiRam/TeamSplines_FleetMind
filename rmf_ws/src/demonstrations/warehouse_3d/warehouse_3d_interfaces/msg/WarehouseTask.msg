# WarehouseTask.msg
# Defines a discrete warehouse movement task

# Priority constants
string PRIORITY_LOW=LOW
string PRIORITY_MEDIUM=MEDIUM
string PRIORITY_HIGH=HIGH
string PRIORITY_CRITICAL=CRITICAL

# Status constants
string STATUS_PENDING=PENDING
string STATUS_ASSIGNED=ASSIGNED
string STATUS_IN_PROGRESS=IN_PROGRESS
string STATUS_COMPLETED=COMPLETED
string STATUS_FAILED=FAILED
string STATUS_CANCELLED=CANCELLED

string task_id
string priority
string package_id
string pickup_location
string destination_location
string status
string assigned_robot

geometry_msgs/Pose pickup_pose
geometry_msgs/Pose destination_pose
builtin_interfaces/Time created_at
