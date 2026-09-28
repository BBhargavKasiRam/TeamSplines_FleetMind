"""
Warehouse Landmark Locations and Waypoints Map
Provides dynamic coordinate lookup for all warehouse zones, racks, packing stations, and docks.
"""

from geometry_msgs.msg import Pose

# Warehouse operational locations with global map coordinates (x, y, z, yaw)
WAREHOUSE_LOCATIONS = {
    # Rack A pickups (Aisle 1 access: Y ~ -6.0)
    "Rack_A_01": {"x": -8.0, "y": -6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_A_02": {"x": -6.0, "y": -6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_A_03": {"x": -4.0, "y": -6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_A_04": {"x": -2.0, "y": -6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # Rack B pickups (Aisle 1 / 2 access: Y ~ -3.0)
    "Rack_B_01": {"x": -8.0, "y": -3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_B_02": {"x": -6.0, "y": -3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_B_03": {"x": -4.0, "y": -3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # Rack C pickups (Aisle 2 / 3 access: Y ~ 0.0)
    "Rack_C_01": {"x": -8.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_C_02": {"x": -6.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_C_06": {"x": -4.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # Rack D pickups (Aisle 3 / 4 access: Y ~ 3.0)
    "Rack_D_01": {"x": -8.0, "y": 3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_D_02": {"x": -6.0, "y": 3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_D_03": {"x": -4.0, "y": 3.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # Rack E pickups (Aisle 4 / 5 access: Y ~ 6.0)
    "Rack_E_01": {"x": -8.0, "y": 6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_E_02": {"x": -6.0, "y": 6.0, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # Rack F pickups (Aisle 5 access: Y ~ 8.5)
    "Rack_F_01": {"x": -8.0, "y": 8.5, "z": 0.0, "yaw": 0.0, "zone": "RACK"},
    "Rack_F_02": {"x": -6.0, "y": 8.5, "z": 0.0, "yaw": 0.0, "zone": "RACK"},

    # East Block Rack Pickups
    "Rack_East_01": {"x": 4.0, "y": -6.0, "z": 0.0, "yaw": 3.1415, "zone": "RACK"},
    "Rack_East_02": {"x": 6.0, "y": -3.0, "z": 0.0, "yaw": 3.1415, "zone": "RACK"},
    "Rack_East_03": {"x": 4.0, "y": 0.0, "z": 0.0, "yaw": 3.1415, "zone": "RACK"},
    "Rack_East_04": {"x": 6.0, "y": 3.0, "z": 0.0, "yaw": 3.1415, "zone": "RACK"},

    # General Storage & Staging
    "Storage_Area": {"x": -1.0, "y": -1.5, "z": 0.0, "yaw": 0.0, "zone": "STORAGE"},
    "Loading_Area": {"x": -11.0, "y": 8.5, "z": 0.0, "yaw": -1.57, "zone": "LOADING"},
    "Loading_Dock": {"x": -12.0, "y": 9.5, "z": 0.0, "yaw": -1.57, "zone": "DOCK"},

    # Dynamic Destinations
    "Packing_01": {"x": -11.0, "y": -9.5, "z": 0.0, "yaw": 1.57, "zone": "PACKING"},
    "Packing_02": {"x": -11.0, "y": -7.5, "z": 0.0, "yaw": 1.57, "zone": "PACKING"},
    "Packing_03": {"x": -11.0, "y": -5.5, "z": 0.0, "yaw": 1.57, "zone": "PACKING"},
    "Shipping_Area": {"x": -13.0, "y": -8.5, "z": 0.0, "yaw": 3.1415, "zone": "PACKING"},
    "Temporary_Storage": {"x": 10.0, "y": 0.0, "z": 0.0, "yaw": 3.1415, "zone": "STORAGE"},
}

PICKUP_LOCATIONS = [
    "Rack_A_01", "Rack_A_02", "Rack_A_03", "Rack_A_04",
    "Rack_B_01", "Rack_B_02", "Rack_B_03",
    "Rack_C_01", "Rack_C_02", "Rack_C_06",
    "Rack_D_01", "Rack_D_02", "Rack_D_03",
    "Rack_E_01", "Rack_E_02",
    "Rack_F_01", "Rack_F_02",
    "Rack_East_01", "Rack_East_02", "Rack_East_03", "Rack_East_04",
    "Loading_Area", "Storage_Area"
]

DESTINATION_LOCATIONS = [
    "Packing_01",
    "Packing_02",
    "Packing_03",
    "Shipping_Area",
    "Loading_Dock",
    "Temporary_Storage"
]

def get_location_pose(loc_name: str) -> Pose:
    pose = Pose()
    if loc_name in WAREHOUSE_LOCATIONS:
        loc = WAREHOUSE_LOCATIONS[loc_name]
        pose.position.x = float(loc['x'])
        pose.position.y = float(loc['y'])
        pose.position.z = float(loc['z'])
        # Yaw to quaternion
        import math
        yaw = loc.get('yaw', 0.0)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
    return pose
