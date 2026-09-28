#!/usr/bin/env bash
# -*- coding: utf-8 -*-
""":"
# Python runner wrapper
exec python3 "$0" "$@"
"""

"""
Traditional Open-RMF Telemetry & REST API Server (rmf_ws_t).
Exposes live telemetry for traditional Open-RMF warehouse robots on http://localhost:8081.
Tracks:
  - Work assigned to each robot (task_id, target waypoints)
  - Distance travelled (Euclidean odometry accumulation)
  - Time taken (active travel time vs standstill schedule wait time)
  - Halts incurred from centralized stop-and-wait traffic negotiation
"""

import os
import sys
import time
import math
import json
import threading
import datetime
import http.server
import socketserver
import uuid
from typing import Dict, Any

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from rmf_fleet_msgs.msg import FleetState, RobotState, RobotMode
    from rmf_task_msgs.msg import ApiRequest
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False


# Data storage for traditional fleet in rmf_ws_t
traditional_fleet_data = {
    'source': 'rmf_ws_t',
    'architecture': 'Traditional Centralized Open-RMF',
    'policy': 'Centralized Traffic Schedule & Reservation Node (Stop-and-Wait)',
    'online': False,
    'last_updated': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    'robots': {
        'tinyRobot1': {
            'id': 'tinyRobot1',
            'fleet': 'tinyRobot',
            'task_id': 'PATROL_LOADING_DOCK',
            'assigned_work': 'loading_dock -> Rack_F_02 -> Rack_E_02 -> tinyRobot1_charger',
            'status_desc': 'TRADITIONAL: Traffic Schedule Negotiation',
            'mode': 'IDLE',
            'mode_code': 0,
            'x': 13.5,
            'y': -28.5,
            'yaw': 0.0,
            'battery': 100.0,
            'distance_m': 0.0,
            'time_taken_sec': 0.0,
            'active_time_sec': 0.0,
            'wait_time_sec': 0.0,
            'halts_count': 0,
            'avg_speed_mps': 0.0,
            'current_speed_mps': 0.0,
            'last_x': 13.5,
            'last_y': -28.5,
            'last_time': time.time(),
            'start_time': None
        },
        'tinyRobot2': {
            'id': 'tinyRobot2',
            'fleet': 'tinyRobot',
            'task_id': 'PATROL_PACKING',
            'assigned_work': 'packing_01 -> Rack_East_04 -> Rack_East_01 -> tinyRobot2_charger',
            'status_desc': 'TRADITIONAL: Traffic Schedule Negotiation',
            'mode': 'IDLE',
            'mode_code': 0,
            'x': 13.5,
            'y': -30.5,
            'yaw': 0.0,
            'battery': 100.0,
            'distance_m': 0.0,
            'time_taken_sec': 0.0,
            'active_time_sec': 0.0,
            'wait_time_sec': 0.0,
            'halts_count': 0,
            'avg_speed_mps': 0.0,
            'current_speed_mps': 0.0,
            'last_x': 13.5,
            'last_y': -30.5,
            'last_time': time.time(),
            'start_time': None
        },
        'tinyRobot3': {
            'id': 'tinyRobot3',
            'fleet': 'tinyRobot',
            'task_id': 'PATROL_RECEIVING',
            'assigned_work': 'loading_area -> Rack_D_02 -> packing_02 -> charger_gamma',
            'status_desc': 'TRADITIONAL: Traffic Schedule Negotiation',
            'mode': 'IDLE',
            'mode_code': 0,
            'x': 13.5,
            'y': -32.5,
            'yaw': 0.0,
            'battery': 100.0,
            'distance_m': 0.0,
            'time_taken_sec': 0.0,
            'active_time_sec': 0.0,
            'wait_time_sec': 0.0,
            'halts_count': 0,
            'avg_speed_mps': 0.0,
            'current_speed_mps': 0.0,
            'last_x': 13.5,
            'last_y': -32.5,
            'last_time': time.time(),
            'start_time': None
        },
        'tinyRobot4': {
            'id': 'tinyRobot4',
            'fleet': 'tinyRobot',
            'task_id': 'PATROL_SHIPPING',
            'assigned_work': 'shipping_area -> Rack_C_02 -> Rack_B_02 -> charger_delta',
            'status_desc': 'TRADITIONAL: Traffic Schedule Negotiation',
            'mode': 'IDLE',
            'mode_code': 0,
            'x': 13.5,
            'y': -34.5,
            'yaw': 0.0,
            'battery': 100.0,
            'distance_m': 0.0,
            'time_taken_sec': 0.0,
            'active_time_sec': 0.0,
            'wait_time_sec': 0.0,
            'halts_count': 0,
            'avg_speed_mps': 0.0,
            'current_speed_mps': 0.0,
            'last_x': 13.5,
            'last_y': -34.5,
            'last_time': time.time(),
            'start_time': None
        },
        'tinyRobot5': {
            'id': 'tinyRobot5',
            'fleet': 'tinyRobot',
            'task_id': 'PATROL_STORAGE',
            'assigned_work': 'packing_03 -> Rack_A_02 -> storage_area -> charger_epsilon',
            'status_desc': 'TRADITIONAL: Traffic Schedule Negotiation',
            'mode': 'IDLE',
            'mode_code': 0,
            'x': 13.5,
            'y': -36.5,
            'yaw': 0.0,
            'battery': 100.0,
            'distance_m': 0.0,
            'time_taken_sec': 0.0,
            'active_time_sec': 0.0,
            'wait_time_sec': 0.0,
            'halts_count': 0,
            'avg_speed_mps': 0.0,
            'current_speed_mps': 0.0,
            'last_x': 13.5,
            'last_y': -36.5,
            'last_time': time.time(),
            'start_time': None
        }
    },
    'summary': {
        'total_distance_m': 0.0,
        'total_time_taken_sec': 0.0,
        'total_wait_time_sec': 0.0,
        'total_halts': 0,
        'avg_fleet_speed_mps': 0.0,
        'active_robots_count': 0,
        'stalled_robots_count': 0
    }
}

data_lock = threading.Lock()
ros_node = None


class TraditionalRmfTelemetryNode(Node):
    """Subscribes to rmf_ws_t ROS 2 topics and updates live metrics."""
    def __init__(self):
        super().__init__('traditional_rmf_telemetry_node')
        self.fleet_sub = self.create_subscription(
            FleetState, '/fleet_states', self.on_fleet_state, 10)

        task_api_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.task_api_pub = self.create_publisher(ApiRequest, '/task_api_requests', task_api_qos)
        self.get_logger().info('Traditional Open-RMF Telemetry Subscriber Active on /fleet_states')

    def on_fleet_state(self, msg: FleetState):
        now = time.time()
        with data_lock:
            traditional_fleet_data['online'] = True
            traditional_fleet_data['last_updated'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            for r in msg.robots:
                name = r.name
                if name not in traditional_fleet_data['robots']:
                    continue

                r_entry = traditional_fleet_data['robots'][name]
                x = float(r.location.x)
                y = float(r.location.y)
                yaw = float(r.location.yaw)
                mode_val = int(r.mode.mode)

                # Initialize start time on first movement
                if r_entry['start_time'] is None:
                    r_entry['start_time'] = now
                    r_entry['last_time'] = now
                    r_entry['last_x'] = x
                    r_entry['last_y'] = y

                # Calculate incremental distance
                dt = max(0.01, now - r_entry['last_time'])
                step = math.hypot(x - r_entry['last_x'], y - r_entry['last_y'])
                if step > 0.01:
                    r_entry['distance_m'] = round(r_entry['distance_m'] + step, 2)
                    r_entry['active_time_sec'] = round(r_entry['active_time_sec'] + dt, 1)
                    speed = step / dt
                else:
                    speed = 0.0

                r_entry['current_speed_mps'] = round(speed, 2)
                r_entry['x'] = round(x, 2)
                r_entry['y'] = round(y, 2)
                r_entry['yaw'] = round(yaw, 2)
                if r_entry.get('distance_m', 0.0) > 0.1:
                    active_s = r_entry.get('active_time_sec', 0.0) + r_entry.get('wait_time_sec', 0.0)
                    trad_drain = (r_entry['distance_m'] * 0.22) + (active_s * 0.03)
                    r_entry['battery'] = max(8.0, round(100.0 - trad_drain, 1))
                else:
                    r_entry['battery'] = round(float(r.battery_percent), 1)
                r_entry['mode_code'] = mode_val

                # Standstill/Wait detection: mode 2 is MODE_PAUSED in Open-RMF
                if mode_val == 2 or (speed < 0.03 and r_entry['distance_m'] > 1.0):
                    r_entry['wait_time_sec'] = round(r_entry['wait_time_sec'] + dt, 1)
                    if speed < 0.03 and not r_entry.get('_was_paused', False):
                        r_entry['halts_count'] += 1
                        r_entry['_was_paused'] = True
                    r_entry['mode'] = 'WAITING_FOR_SCHEDULE'
                    r_entry['status_desc'] = 'YIELDING: Centralized Schedule Wait'
                else:
                    r_entry['_was_paused'] = False
                    r_entry['mode'] = 'MOVING' if speed >= 0.05 else ('CHARGING' if mode_val == 3 else 'IDLE')
                    r_entry['status_desc'] = 'NAVIGATING (Fixed Waypoints)' if speed >= 0.05 else 'AT CHARGER'

                # Total time
                if r_entry['start_time']:
                    r_entry['time_taken_sec'] = round(now - r_entry['start_time'], 1)
                    if r_entry['time_taken_sec'] > 0:
                        r_entry['avg_speed_mps'] = round(r_entry['distance_m'] / r_entry['time_taken_sec'], 2)

                if r.task_id and r.task_id != '':
                    r_entry['task_id'] = r.task_id

                r_entry['last_x'] = x
                r_entry['last_y'] = y
                r_entry['last_time'] = now

            # Update summary aggregates
            robots_list = list(traditional_fleet_data['robots'].values())
            tot_dist = sum(r['distance_m'] for r in robots_list)
            tot_time = max(r['time_taken_sec'] for r in robots_list) if robots_list else 0.0
            tot_wait = sum(r['wait_time_sec'] for r in robots_list)
            tot_halts = sum(r['halts_count'] for r in robots_list)
            active_cnt = sum(1 for r in robots_list if r['mode'] == 'MOVING')
            stalled_cnt = sum(1 for r in robots_list if r['mode'] == 'WAITING_FOR_SCHEDULE')

            traditional_fleet_data['summary'] = {
                'total_distance_m': round(tot_dist, 1),
                'total_time_taken_sec': round(tot_time, 1),
                'total_wait_time_sec': round(tot_wait, 1),
                'total_halts': tot_halts,
                'avg_fleet_speed_mps': round(tot_dist / max(1.0, tot_time), 2) if tot_time > 0 else 0.0,
                'active_robots_count': active_cnt,
                'stalled_robots_count': stalled_cnt
            }


def publish_traditional_openrmf_task(robot_name: str, places: list, rounds: int = 1):
    global ros_node
    if not ros_node or not hasattr(ros_node, 'task_api_pub'):
        return False
    try:
        msg = ApiRequest()
        msg.request_id = f"trad_{uuid.uuid4().hex[:8]}"
        payload = {
            "type": "robot_task_request",
            "robot": robot_name,
            "fleet": "tinyRobot",
            "request": {
                "unix_millis_request_time": 0,
                "unix_millis_earliest_start_time": 0,
                "requester": "traditional_telemetry_console",
                "category": "patrol",
                "fleet_name": "tinyRobot",
                "description": {
                    "places": places,
                    "rounds": rounds
                }
            }
        }
        msg.json_msg = json.dumps(payload)
        ros_node.task_api_pub.publish(msg)
        print(f"[Trad Open-RMF Task] Dispatched {msg.request_id} to {robot_name}: {places}")
        return True
    except Exception as e:
        print(f"[Trad Open-RMF Task Error] {e}")
        return False


class TraditionalHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path in ['/api/status', '/api/robots', '/api/telemetry']:
            with data_lock:
                payload = json.dumps(traditional_fleet_data, indent=2).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(payload)
            return

        # Simple health check endpoint
        if self.path == '/api/health':
            res = json.dumps({'status': 'ok', 'source': 'rmf_ws_t', 'online': traditional_fleet_data['online']}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(res)
            return

        self.send_error(404, "Endpoint Not Found")

    def do_POST(self):
        content_len = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_len).decode('utf-8') if content_len > 0 else '{}'
        try:
            req_data = json.loads(body)
        except Exception:
            req_data = {}

        if self.path == '/api/reset_metrics':
            with data_lock:
                for r in traditional_fleet_data['robots'].values():
                    r['distance_m'] = 0.0
                    r['time_taken_sec'] = 0.0
                    r['active_time_sec'] = 0.0
                    r['wait_time_sec'] = 0.0
                    r['halts_count'] = 0
                    r['start_time'] = None
            res = json.dumps({'status': 'reset_complete'}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(res)
            return

        elif self.path in ['/api/run_benchmark', '/api/dispatch_task']:
            scenario = req_data.get('mission', req_data.get('scenario', 'benchmark'))
            PRESETS = {
                'benchmark': {
                    'tinyRobot1': ['loading_dock', 'Rack_F_02', 'Rack_E_02', 'tinyRobot1_charger'],
                    'tinyRobot2': ['packing_01', 'Rack_East_04', 'Rack_East_01', 'tinyRobot2_charger'],
                    'tinyRobot3': ['loading_area', 'Rack_D_02', 'packing_02', 'charger_gamma'],
                    'tinyRobot4': ['shipping_area', 'Rack_C_02', 'Rack_B_02', 'charger_delta'],
                    'tinyRobot5': ['packing_03', 'Rack_A_02', 'storage_area', 'charger_epsilon']
                },
                'putaway': {
                    'tinyRobot1': ['loading_dock', 'Rack_F_02', 'tinyRobot1_charger'],
                    'tinyRobot2': ['loading_dock', 'Rack_E_02', 'tinyRobot2_charger'],
                    'tinyRobot3': ['loading_dock', 'Rack_D_02', 'charger_gamma'],
                    'tinyRobot4': ['loading_dock', 'Rack_C_02', 'charger_delta'],
                    'tinyRobot5': ['loading_dock', 'Rack_B_02', 'charger_epsilon']
                },
                'sorting': {
                    'tinyRobot1': ['packing_01', 'Rack_East_04', 'tinyRobot1_charger'],
                    'tinyRobot2': ['packing_02', 'Rack_East_01', 'tinyRobot2_charger'],
                    'tinyRobot3': ['packing_03', 'Rack_D_02', 'charger_gamma'],
                    'tinyRobot4': ['packing_01', 'Rack_C_02', 'charger_delta'],
                    'tinyRobot5': ['packing_02', 'Rack_A_02', 'charger_epsilon']
                },
                'head_on': {
                    'tinyRobot1': ['packing_01', 'tinyRobot1_charger'],
                    'tinyRobot4': ['loading_dock', 'charger_delta']
                }
            }
            places_map = PRESETS.get(scenario, PRESETS['benchmark'])
            target_robot = req_data.get('robot', 'all')
            targets = list(places_map.keys()) if target_robot == 'all' else [target_robot]

            now = time.time()
            with data_lock:
                for r_name in targets:
                    if r_name in traditional_fleet_data['robots']:
                        r = traditional_fleet_data['robots'][r_name]
                        r['distance_m'] = 0.0
                        r['time_taken_sec'] = 0.0
                        r['active_time_sec'] = 0.0
                        r['wait_time_sec'] = 0.0
                        r['halts_count'] = 0
                        r['start_time'] = now
                        r['last_time'] = now
                        r['assigned_work'] = ' -> '.join(places_map[r_name])
                        r['task_id'] = f"PATROL_{scenario.upper()}"
                        r['status_desc'] = "TRADITIONAL: Traffic Schedule Negotiation"

            for r_name in targets:
                publish_traditional_openrmf_task(r_name, places_map[r_name], 1)

            res = json.dumps({
                'status': 'dispatched',
                'workspace': 'rmf_ws_t',
                'scenario': scenario,
                'targets': targets,
                'message': f"Traditional Open-RMF task dispatched for {targets} in scenario [{scenario}]."
            }).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(res)
            return

        elif self.path == '/api/return_to_charger':
            CHARGERS = {
                'tinyRobot1': 'tinyRobot1_charger',
                'tinyRobot2': 'tinyRobot2_charger',
                'tinyRobot3': 'charger_gamma',
                'tinyRobot4': 'charger_delta',
                'tinyRobot5': 'charger_epsilon'
            }
            target_robot = req_data.get('robot', 'all')
            targets = list(CHARGERS.keys()) if target_robot == 'all' else [target_robot]
            for r_name in targets:
                charger_pt = CHARGERS.get(r_name)
                if charger_pt:
                    publish_traditional_openrmf_task(r_name, [charger_pt], 1)
            res = json.dumps({'status': 'returning', 'targets': targets}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(res)
            return

        self.send_error(404, "Endpoint Not Found")


def run_http_server(port=8081):
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    httpd = ThreadedHTTPServer(("", port), TraditionalHTTPRequestHandler)
    print(f"[*] Traditional Open-RMF Telemetry API Server running on: http://localhost:{port}")
    httpd.serve_forever()


def main():
    global ros_node
    # Start HTTP server on port 8081
    http_thread = threading.Thread(target=run_http_server, args=(8081,), daemon=True)
    http_thread.start()

    # Start ROS 2 node if rclpy is available
    if RCLPY_AVAILABLE:
        try:
            rclpy.init()
            ros_node = TraditionalRmfTelemetryNode()
            rclpy.spin(ros_node)
        except KeyboardInterrupt:
            pass
        finally:
            if ros_node:
                ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        print("[!] rclpy not found, running HTTP mock server only.")
        while True:
            time.sleep(1)


if __name__ == '__main__':
    main()
