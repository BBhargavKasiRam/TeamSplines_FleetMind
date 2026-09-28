# WarehouseLocation.msg
# Landmark location in warehouse 3D environment

string ZONE_STORAGE=STORAGE
string ZONE_RACK=RACK
string ZONE_SHELF=SHELF
string ZONE_PALLET=PALLET
string ZONE_LOADING=LOADING
string ZONE_UNLOADING=UNLOADING
string ZONE_PACKING=PACKING
string ZONE_CHARGING=CHARGING
string ZONE_DOCK=DOCK
string ZONE_CORRIDOR=CORRIDOR

string location_id
string zone_type
geometry_msgs/Pose pose
bool is_occupied
string occupant_id
