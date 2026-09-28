#!/usr/bin/env python3
"""
Warehouse Marker Publisher
Publishes comprehensive presentation-ready RViz 3D markers:
- Floating 3D Robot Name, Battery, and Status Badges
- Dynamic Conflict / Deadlock / Yield visual highlights
- Glowing forward trajectory ribbons and trailing path ribbons
- Goal waypoint target discs
- Biscuit / light cardboard brown 3D package boxes (96 populated units)
- Wooden pallets at staging locations
- Operational Zone 3D Wireframes & Floating Zone Title Signs
- High-fidelity Operations Task Billboards with dark tinted glass panels & priority headers
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from warehouse_3d_interfaces.msg import RobotState, WarehouseTask

ROBOT_COLORS = {
    "robot_alpha":   {"r": 0.12, "g": 0.45, "b": 0.95, "name": "Robot Alpha"},
    "robot_beta":    {"r": 0.95, "g": 0.15, "b": 0.15, "name": "Robot Beta"},
    "robot_gamma":   {"r": 0.15, "g": 0.85, "b": 0.25, "name": "Robot Gamma"},
    "robot_delta":   {"r": 0.95, "g": 0.85, "b": 0.10, "name": "Robot Delta"},
    "robot_epsilon": {"r": 0.75, "g": 0.18, "b": 0.92, "name": "Robot Epsilon"},
}

PRIORITY_COLORS = {
    "CRITICAL": {"r": 1.0, "g": 0.05, "b": 0.05},
    "HIGH":     {"r": 1.0, "g": 0.55, "b": 0.05},
    "MEDIUM":   {"r": 0.9, "g": 0.85, "b": 0.10},
    "LOW":      {"r": 0.2, "g": 0.80, "b": 0.20},
}

SPAWN_POSES = {
    "robot_alpha":   {"x": -12.5, "y": -2.0, "z": 0.0},
    "robot_beta":    {"x": -12.5, "y": -1.0, "z": 0.0},
    "robot_gamma":   {"x": -12.5, "y":  0.0, "z": 0.0},
    "robot_delta":   {"x": -12.5, "y":  1.0, "z": 0.0},
    "robot_epsilon": {"x": -12.5, "y":  2.0, "z": 0.0},
}

def get_all_warehouse_boxes():
    """Generates 96 box coordinates across all 12 rack rows and staging locations."""
    boxes = []
    pid_counter = 1
    # West Racks (Rack_A to Rack_F)
    west_rack_ys = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
    west_rack_xs = [-9.0, -7.5, -6.0, -4.5, -3.0]
    for y_coord in west_rack_ys:
        for x_coord in west_rack_xs:
            boxes.append((f"P{pid_counter:03d}", x_coord, y_coord, 0.82))
            pid_counter += 1
        for x_coord in [west_rack_xs[0], west_rack_xs[2], west_rack_xs[4]]:
            boxes.append((f"P{pid_counter:03d}", x_coord, y_coord, 1.82))
            pid_counter += 1

    # East Racks (Rack_A_East to Rack_F_East)
    east_rack_ys = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
    east_rack_xs = [3.0, 4.5, 6.0, 7.5, 9.0]
    for y_coord in east_rack_ys:
        for x_coord in east_rack_xs:
            boxes.append((f"P{pid_counter:03d}", x_coord, y_coord, 0.82))
            pid_counter += 1
        for x_coord in [east_rack_xs[1], east_rack_xs[3]]:
            boxes.append((f"P{pid_counter:03d}", x_coord, y_coord, 1.82))
            pid_counter += 1

    # Pallets at Docks & Packing
    for px, py in [(-12.0, 9.5), (-12.0, 7.5), (-12.0, -9.5), (-12.0, -7.5), (12.0, -6.0), (12.0, 6.0)]:
        boxes.append((f"P{pid_counter:03d}", px, py, 0.32))
        pid_counter += 1
    return boxes

class WarehouseMarkerPublisher(Node):
    def __init__(self):
        super().__init__('warehouse_marker_publisher')
        self.get_logger().info("Initializing Warehouse 3D Presentation Marker Visualization...")

        self.marker_pub = self.create_publisher(MarkerArray, '/warehouse/markers', 10)

        # State storage
        self.robot_states = {}
        self.robot_history = {ns: [] for ns in ROBOT_COLORS}
        self.robot_waypoints = {}
        self.robot_active_paths = {ns: [] for ns in ROBOT_COLORS}
        self.active_tasks = {}

        # Subscriptions
        for ns in ROBOT_COLORS:
            self.create_subscription(
                RobotState,
                f'/{ns}/robot_state',
                lambda msg, r_ns=ns: self.robot_state_callback(r_ns, msg),
                10
            )
            self.create_subscription(
                PoseStamped,
                f'/logic/{ns}/waypoint',
                lambda msg, r_ns=ns: self.waypoint_callback(r_ns, msg),
                10
            )
            self.create_subscription(
                Path,
                f'/logic/{ns}/path',
                lambda msg, r_ns=ns: self.path_callback(r_ns, msg),
                10
            )

        self.create_subscription(
            WarehouseTask,
            '/warehouse/task_stream',
            self.task_callback,
            20
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_markers)

    def robot_state_callback(self, ns, msg: RobotState):
        self.robot_states[ns] = msg
        p = msg.current_pose.position
        hist = self.robot_history[ns]
        if not hist or math.hypot(p.x - hist[-1].x, p.y - hist[-1].y) > 0.15:
            pt = Point()
            pt.x = p.x
            pt.y = p.y
            pt.z = 0.05
            hist.append(pt)
            if len(hist) > 100:
                hist.pop(0)

        # Prune reached waypoints in active path so path ahead stays updated
        if ns in self.robot_active_paths and self.robot_active_paths[ns]:
            pts = self.robot_active_paths[ns]
            while pts and math.hypot(p.x - pts[0].x, p.y - pts[0].y) < 0.45:
                pts.pop(0)

    def path_callback(self, ns, msg: Path):
        self.robot_active_paths[ns] = [p.pose.position for p in msg.poses]

    def waypoint_callback(self, ns, msg: PoseStamped):
        self.robot_waypoints[ns] = msg.pose.position

    def task_callback(self, msg: WarehouseTask):
        self.active_tasks[msg.task_id] = msg

    def publish_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        # =====================================================================
        # 1. 3D AMR Robots, Identity Masts, Beacons, Badges & Conflict Halos
        # =====================================================================
        for ns, cinfo in ROBOT_COLORS.items():
            r_state = self.robot_states.get(ns)
            spawn = SPAWN_POSES.get(ns, {"x": 0.0, "y": 0.0, "z": 0.0})
            rx = r_state.current_pose.position.x if r_state else spawn["x"]
            ry = r_state.current_pose.position.y if r_state else spawn["y"]
            rz = r_state.current_pose.position.z if r_state else spawn["z"]
            status = r_state.status if r_state else "STANDBY"
            batt = f"{r_state.battery_percentage:.0f}%" if r_state else "100%"
            orientation = r_state.current_pose.orientation if r_state else None

            # 1a. 3D AMR Main Chassis Box
            bm = Marker()
            bm.header.frame_id = 'map'
            bm.header.stamp = now
            bm.ns = 'robot_bodies'
            bm.id = marker_id
            marker_id += 1
            bm.type = Marker.CUBE
            bm.action = Marker.ADD
            bm.pose.position.x = rx
            bm.pose.position.y = ry
            bm.pose.position.z = rz + 0.13
            if orientation and (orientation.w != 0.0 or orientation.z != 0.0):
                bm.pose.orientation = orientation
            else:
                bm.pose.orientation.w = 1.0
            bm.scale.x = 0.72
            bm.scale.y = 0.52
            bm.scale.z = 0.26
            bm.color.r = float(cinfo['r'])
            bm.color.g = float(cinfo['g'])
            bm.color.b = float(cinfo['b'])
            bm.color.a = 0.95
            markers.markers.append(bm)

            # 1b. Glowing Top Status Beacon Sphere
            bcon = Marker()
            bcon.header.frame_id = 'map'
            bcon.header.stamp = now
            bcon.ns = 'robot_beacons'
            bcon.id = marker_id
            marker_id += 1
            bcon.type = Marker.SPHERE
            bcon.action = Marker.ADD
            bcon.pose.position.x = rx
            bcon.pose.position.y = ry
            bcon.pose.position.z = rz + 0.35
            bcon.pose.orientation.w = 1.0
            bcon.scale.x = 0.16
            bcon.scale.y = 0.16
            bcon.scale.z = 0.16
            if status == "MOVING":
                bcon.color.r, bcon.color.g, bcon.color.b, bcon.color.a = 0.1, 1.0, 0.2, 1.0
            elif status == "YIELDING":
                bcon.color.r, bcon.color.g, bcon.color.b, bcon.color.a = 1.0, 0.75, 0.05, 1.0
            elif status == "DEADLOCKED":
                bcon.color.r, bcon.color.g, bcon.color.b, bcon.color.a = 1.0, 0.05, 0.05, 1.0
            else:
                bcon.color.r, bcon.color.g, bcon.color.b, bcon.color.a = 0.15, 0.85, 1.0, 1.0
            markers.markers.append(bcon)

            # 1c. Forward Heading Direction Arrow
            arr = Marker()
            arr.header.frame_id = 'map'
            arr.header.stamp = now
            arr.ns = 'robot_headings'
            arr.id = marker_id
            marker_id += 1
            arr.type = Marker.ARROW
            arr.action = Marker.ADD
            arr.pose.position.x = rx
            arr.pose.position.y = ry
            arr.pose.position.z = rz + 0.28
            if orientation and (orientation.w != 0.0 or orientation.z != 0.0):
                arr.pose.orientation = orientation
            else:
                arr.pose.orientation.w = 1.0
            arr.scale.x = 0.55
            arr.scale.y = 0.12
            arr.scale.z = 0.10
            arr.color.r = 1.0
            arr.color.g = 1.0
            arr.color.b = 1.0
            arr.color.a = 0.95
            markers.markers.append(arr)

            # 1d. Text Label Marker
            tm = Marker()
            tm.header.frame_id = 'map'
            tm.header.stamp = now
            tm.ns = 'robot_labels'
            tm.id = marker_id
            marker_id += 1
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = rx
            tm.pose.position.y = ry
            tm.pose.position.z = rz + 0.75
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.35
            tm.color.r = float(cinfo['r'])
            tm.color.g = float(cinfo['g'])
            tm.color.b = float(cinfo['b'])
            tm.color.a = 1.0
            tm.text = f"{cinfo['name']}\n[{status}] Batt:{batt}"
            markers.markers.append(tm)

            # 1e. Robot Disc Base Glow
            gm = Marker()
            gm.header.frame_id = 'map'
            gm.header.stamp = now
            gm.ns = 'robot_glow'
            gm.id = marker_id
            marker_id += 1
            gm.type = Marker.CYLINDER
            gm.action = Marker.ADD
            gm.pose.position.x = rx
            gm.pose.position.y = ry
            gm.pose.position.z = 0.02
            gm.pose.orientation.w = 1.0
            gm.scale.x = 1.1
            gm.scale.y = 1.1
            gm.scale.z = 0.02
            gm.color.r = float(cinfo['r'])
            gm.color.g = float(cinfo['g'])
            gm.color.b = float(cinfo['b'])
            gm.color.a = 0.55
            markers.markers.append(gm)

            # 1f. Vertical Identifier Mast Line at Back of Robot ("small line back of it to identify where it is")
            yaw = 0.0
            if orientation:
                siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
                cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
            back_x = rx - 0.28 * math.cos(yaw)
            back_y = ry - 0.28 * math.sin(yaw)

            # Mast vertical pole
            mast = Marker()
            mast.header.frame_id = 'map'
            mast.header.stamp = now
            mast.ns = 'robot_masts'
            mast.id = marker_id
            marker_id += 1
            mast.type = Marker.CYLINDER
            mast.action = Marker.ADD
            mast.pose.position.x = back_x
            mast.pose.position.y = back_y
            mast.pose.position.z = rz + 0.42
            mast.pose.orientation.w = 1.0
            mast.scale.x = 0.03
            mast.scale.y = 0.03
            mast.scale.z = 0.65
            mast.color.r = 0.15
            mast.color.g = 0.18
            mast.color.b = 0.22
            mast.color.a = 1.0
            markers.markers.append(mast)

            # Mast top glowing identity orb
            orb = Marker()
            orb.header.frame_id = 'map'
            orb.header.stamp = now
            orb.ns = 'robot_masts'
            orb.id = marker_id
            marker_id += 1
            orb.type = Marker.SPHERE
            orb.action = Marker.ADD
            orb.pose.position.x = back_x
            orb.pose.position.y = back_y
            orb.pose.position.z = rz + 0.76
            orb.pose.orientation.w = 1.0
            orb.scale.x = 0.15
            orb.scale.y = 0.15
            orb.scale.z = 0.15
            orb.color.r = float(cinfo['r'])
            orb.color.g = float(cinfo['g'])
            orb.color.b = float(cinfo['b'])
            orb.color.a = 1.0
            markers.markers.append(orb)

            # 1g. Conflict / Deadlock / Yield Dynamic Highlight Rings
            if status == "DEADLOCKED":
                # Pulsing red danger halo ring
                d_halo = Marker()
                d_halo.header.frame_id = 'map'
                d_halo.header.stamp = now
                d_halo.ns = 'conflict_alerts'
                d_halo.id = marker_id
                marker_id += 1
                d_halo.type = Marker.CYLINDER
                d_halo.action = Marker.ADD
                d_halo.pose.position.x = rx
                d_halo.pose.position.y = ry
                d_halo.pose.position.z = 0.04
                d_halo.pose.orientation.w = 1.0
                d_halo.scale.x = 1.6
                d_halo.scale.y = 1.6
                d_halo.scale.z = 0.05
                d_halo.color.r, d_halo.color.g, d_halo.color.b, d_halo.color.a = 1.0, 0.05, 0.05, 0.75
                markers.markers.append(d_halo)

                # Floating alert text
                d_txt = Marker()
                d_txt.header.frame_id = 'map'
                d_txt.header.stamp = now
                d_txt.ns = 'conflict_alerts'
                d_txt.id = marker_id
                marker_id += 1
                d_txt.type = Marker.TEXT_VIEW_FACING
                d_txt.action = Marker.ADD
                d_txt.pose.position.x = rx
                d_txt.pose.position.y = ry
                d_txt.pose.position.z = rz + 1.15
                d_txt.pose.orientation.w = 1.0
                d_txt.scale.z = 0.26
                d_txt.color.r, d_txt.color.g, d_txt.color.b, d_txt.color.a = 1.0, 0.15, 0.15, 1.0
                d_txt.text = "⚠️ DEADLOCK DETECTED"
                markers.markers.append(d_txt)

            elif status == "YIELDING":
                # Amber yielding halo ring
                y_halo = Marker()
                y_halo.header.frame_id = 'map'
                y_halo.header.stamp = now
                y_halo.ns = 'conflict_alerts'
                y_halo.id = marker_id
                marker_id += 1
                y_halo.type = Marker.CYLINDER
                y_halo.action = Marker.ADD
                y_halo.pose.position.x = rx
                y_halo.pose.position.y = ry
                y_halo.pose.position.z = 0.04
                y_halo.pose.orientation.w = 1.0
                y_halo.scale.x = 1.4
                y_halo.scale.y = 1.4
                y_halo.scale.z = 0.04
                y_halo.color.r, y_halo.color.g, y_halo.color.b, y_halo.color.a = 1.0, 0.70, 0.05, 0.65
                markers.markers.append(y_halo)

                y_txt = Marker()
                y_txt.header.frame_id = 'map'
                y_txt.header.stamp = now
                y_txt.ns = 'conflict_alerts'
                y_txt.id = marker_id
                marker_id += 1
                y_txt.type = Marker.TEXT_VIEW_FACING
                y_txt.action = Marker.ADD
                y_txt.pose.position.x = rx
                y_txt.pose.position.y = ry
                y_txt.pose.position.z = rz + 1.15
                y_txt.pose.orientation.w = 1.0
                y_txt.scale.z = 0.22
                y_txt.color.r, y_txt.color.g, y_txt.color.b, y_txt.color.a = 1.0, 0.75, 0.1, 1.0
                y_txt.text = "⏳ YIELDING (PRIORITY HOLD)"
                markers.markers.append(y_txt)

            # =================================================================
            # 2. Paths, Trailing Tracks & Waypoint Targets
            # =================================================================
            # 2a. Forward Path Ahead of Robot ("path like where the robot is going")
            active_pts = self.robot_active_paths.get(ns, [])
            wp = self.robot_waypoints.get(ns)
            fwd_pts = []
            cur_pt = Point()
            cur_pt.x = rx
            cur_pt.y = ry
            cur_pt.z = rz + 0.10
            fwd_pts.append(cur_pt)

            if active_pts:
                for ap in active_pts:
                    p = Point()
                    p.x = ap.x
                    p.y = ap.y
                    p.z = 0.10
                    fwd_pts.append(p)
            elif wp:
                p = Point()
                p.x = wp.x
                p.y = wp.y
                p.z = 0.10
                fwd_pts.append(p)

            if len(fwd_pts) > 1:
                fwd_m = Marker()
                fwd_m.header.frame_id = 'map'
                fwd_m.header.stamp = now
                fwd_m.ns = 'robot_forward_paths'
                fwd_m.id = marker_id
                marker_id += 1
                fwd_m.type = Marker.LINE_STRIP
                fwd_m.action = Marker.ADD
                fwd_m.pose.orientation.w = 1.0
                fwd_m.scale.x = 0.08 # clear 8cm wide glowing forward path ribbon
                fwd_m.color.r = float(cinfo['r'])
                fwd_m.color.g = float(cinfo['g'])
                fwd_m.color.b = float(cinfo['b'])
                fwd_m.color.a = 0.95
                fwd_m.points = fwd_pts
                markers.markers.append(fwd_m)

            # 2b. Trailing Track Behind Robot
            hist = self.robot_history[ns]
            if len(hist) > 1:
                pm = Marker()
                pm.header.frame_id = 'map'
                pm.header.stamp = now
                pm.ns = 'robot_tracks'
                pm.id = marker_id
                marker_id += 1
                pm.type = Marker.LINE_STRIP
                pm.action = Marker.ADD
                pm.pose.orientation.w = 1.0
                pm.scale.x = 0.04 # 4cm trail ribbon
                pm.color.r = float(cinfo['r'])
                pm.color.g = float(cinfo['g'])
                pm.color.b = float(cinfo['b'])
                pm.color.a = 0.50
                pm.points = hist
                markers.markers.append(pm)

            # 2c. Active Goal Waypoint Target Marker
            if wp:
                wm = Marker()
                wm.header.frame_id = 'map'
                wm.header.stamp = now
                wm.ns = 'robot_waypoints'
                wm.id = marker_id
                marker_id += 1
                wm.type = Marker.CYLINDER
                wm.action = Marker.ADD
                wm.pose.position.x = wp.x
                wm.pose.position.y = wp.y
                wm.pose.position.z = 0.05
                wm.pose.orientation.w = 1.0
                wm.scale.x = 0.45
                wm.scale.y = 0.45
                wm.scale.z = 0.08
                wm.color.r = float(cinfo['r'])
                wm.color.g = float(cinfo['g'])
                wm.color.b = float(cinfo['b'])
                wm.color.a = 0.85
                markers.markers.append(wm)

        # =====================================================================
        # 3. Biscuit-Colored Cardboard Packages (All 96 Populated Boxes)
        # =====================================================================
        box_coords = get_all_warehouse_boxes()
        for pid, bx, by, bz in box_coords:
            bm = Marker()
            bm.header.frame_id = 'map'
            bm.header.stamp = now
            bm.ns = 'packages'
            bm.id = marker_id
            marker_id += 1
            bm.type = Marker.CUBE
            bm.action = Marker.ADD
            bm.pose.position.x = bx
            bm.pose.position.y = by
            bm.pose.position.z = bz
            bm.pose.orientation.w = 1.0
            bm.scale.x = 0.42
            bm.scale.y = 0.40
            bm.scale.z = 0.35
            # Biscuit / light cardboard brown
            bm.color.r = 0.82
            bm.color.g = 0.69
            bm.color.b = 0.51
            bm.color.a = 1.0
            markers.markers.append(bm)

            # Label on box
            bl = Marker()
            bl.header.frame_id = 'map'
            bl.header.stamp = now
            bl.ns = 'package_labels'
            bl.id = marker_id
            marker_id += 1
            bl.type = Marker.TEXT_VIEW_FACING
            bl.action = Marker.ADD
            bl.pose.position.x = bx
            bl.pose.position.y = by
            bl.pose.position.z = bz + 0.3
            bl.pose.orientation.w = 1.0
            bl.scale.z = 0.16
            bl.color.r = 0.1
            bl.color.g = 0.1
            bl.color.b = 0.1
            bl.color.a = 1.0
            bl.text = pid
            markers.markers.append(bl)

        # =====================================================================
        # 4. Pallets at Staging Docks & Packing
        # =====================================================================
        staging_pallets = [
            (-12.0, 9.5), (-12.0, 7.5),
            (-12.0, -9.5), (-12.0, -7.5),
            (12.0, -6.0), (12.0, 6.0)
        ]
        for pi, (px, py) in enumerate(staging_pallets):
            pal = Marker()
            pal.header.frame_id = 'map'
            pal.header.stamp = now
            pal.ns = 'pallets'
            pal.id = marker_id
            marker_id += 1
            pal.type = Marker.CUBE
            pal.action = Marker.ADD
            pal.pose.position.x = px
            pal.pose.position.y = py
            pal.pose.position.z = 0.075
            pal.pose.orientation.w = 1.0
            pal.scale.x = 1.2
            pal.scale.y = 0.8
            pal.scale.z = 0.15
            pal.color.r, pal.color.g, pal.color.b, pal.color.a = 0.58, 0.44, 0.28, 1.0
            markers.markers.append(pal)

        # =====================================================================
        # 5. Operational Zones Wireframes & Floating Title Billboards
        # =====================================================================
        zones = [
            ("ZONE A: INBOUND RECEIVING DOCK", -12.0, 8.5, 2.8, 0.2, 0.85, 0.35),
            ("ZONE B: ASRS STORAGE RACKS",      -0.5, 0.0, 3.4, 0.2, 0.55, 0.95),
            ("ZONE C: OUTBOUND PACKING",      -12.0, -8.5, 2.8, 0.95, 0.55, 0.10),
            ("ZONE D: AMR CHARGING DEPOT",     -12.5, 0.0, 2.0, 0.15, 0.85, 0.95),
        ]
        for ztitle, zx, zy, zz, zr, zg, zb in zones:
            zt = Marker()
            zt.header.frame_id = 'map'
            zt.header.stamp = now
            zt.ns = 'warehouse_zones'
            zt.id = marker_id
            marker_id += 1
            zt.type = Marker.TEXT_VIEW_FACING
            zt.action = Marker.ADD
            zt.pose.position.x = zx
            zt.pose.position.y = zy
            zt.pose.position.z = zz
            zt.pose.orientation.w = 1.0
            zt.scale.z = 0.32
            zt.color.r = zr
            zt.color.g = zg
            zt.color.b = zb
            zt.color.a = 0.95
            zt.text = ztitle
            markers.markers.append(zt)

        # =====================================================================
        # 6. Active Task Billboard Overlays (High-Fidelity Operations Scoreboard)
        # =====================================================================
        sorted_tasks = sorted(self.active_tasks.values(), key=lambda t: t.task_id)[:8]
        board_x = -13.5
        board_y = -11.5
        for idx, task in enumerate(sorted_tasks):
            cx = board_x + (idx % 4) * 2.5
            cy = board_y - (idx // 4) * 1.5
            cz = 2.2

            col = PRIORITY_COLORS.get(task.priority, {"r": 1.0, "g": 1.0, "b": 1.0})

            # Dark tinted glass panel backing
            panel = Marker()
            panel.header.frame_id = 'map'
            panel.header.stamp = now
            panel.ns = 'task_billboard_panels'
            panel.id = marker_id
            marker_id += 1
            panel.type = Marker.CUBE
            panel.action = Marker.ADD
            panel.pose.position.x = cx
            panel.pose.position.y = cy
            panel.pose.position.z = cz
            panel.pose.orientation.w = 1.0
            panel.scale.x = 2.3
            panel.scale.y = 0.04
            panel.scale.z = 1.2
            panel.color.r, panel.color.g, panel.color.b, panel.color.a = 0.08, 0.10, 0.14, 0.88
            markers.markers.append(panel)

            # Priority top header strip
            hdr = Marker()
            hdr.header.frame_id = 'map'
            hdr.header.stamp = now
            hdr.ns = 'task_billboard_headers'
            hdr.id = marker_id
            marker_id += 1
            hdr.type = Marker.CUBE
            hdr.action = Marker.ADD
            hdr.pose.position.x = cx
            hdr.pose.position.y = cy
            hdr.pose.position.z = cz + 0.58
            hdr.pose.orientation.w = 1.0
            hdr.scale.x = 2.3
            hdr.scale.y = 0.05
            hdr.scale.z = 0.08
            hdr.color.r = float(col['r'])
            hdr.color.g = float(col['g'])
            hdr.color.b = float(col['b'])
            hdr.color.a = 1.0
            markers.markers.append(hdr)

            # Formatted Task Information Text
            tb = Marker()
            tb.header.frame_id = 'map'
            tb.header.stamp = now
            tb.ns = 'task_billboard'
            tb.id = marker_id
            marker_id += 1
            tb.type = Marker.TEXT_VIEW_FACING
            tb.action = Marker.ADD
            tb.pose.position.x = cx
            tb.pose.position.y = cy
            tb.pose.position.z = cz
            tb.pose.orientation.w = 1.0
            tb.scale.z = 0.18
            tb.color.r = float(col['r'])
            tb.color.g = float(col['g'])
            tb.color.b = float(col['b'])
            tb.color.a = 1.0
            tb.text = (
                f"[{task.task_id}] {task.priority}\n"
                f"Pkg: {task.package_id}\n"
                f"Pickup: {task.pickup_location}\n"
                f"Dest: {task.destination_location}\n"
                f"Robot: {task.assigned_robot or 'PENDING'}\n"
                f"Status: {task.status}"
            )
            markers.markers.append(tb)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
