# PackageState.msg
# Real-time state of warehouse cardboard package

string STATUS_IN_STORAGE=IN_STORAGE
string STATUS_RESERVED=RESERVED
string STATUS_IN_TRANSIT=IN_TRANSIT
string STATUS_DELIVERED=DELIVERED

string SIZE_SMALL=SMALL
string SIZE_MEDIUM=MEDIUM
string SIZE_LARGE=LARGE

string package_id
string current_location
string status
geometry_msgs/Pose pose
float32 weight_kg
string size_category
string assigned_task_id
string carrier_robot
