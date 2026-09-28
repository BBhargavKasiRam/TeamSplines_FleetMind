#!/usr/bin/env python3
"""
Dynamic Aisle Blockage Injection Tool for Live SIH Demonstrations.
Publishes a blockage event to trigger Edge Perception and Decentralized Task Auction (CNP).
Usage:
  ros2 run edge_fleet_bringup inject_blockage.py --robot robot_1 --blocked true
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description="Inject dynamic aisle blockage for BEL SIH demo")
    parser.add_argument('--robot', type=str, default='robot_1', help='Target robot namespace')
    parser.add_argument('--blocked', type=str, default='true', help='true to block, false to clear')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = Node('blockage_injector')
    topic = f'/{args.robot}/simulate_aisle_blockage'
    pub = node.create_publisher(Bool, topic, 10)

    is_blocked = args.blocked.lower() in ('true', '1', 'yes')

    # Allow topic discovery
    import time
    time.sleep(0.5)

    msg = Bool()
    msg.data = is_blocked
    pub.publish(msg)

    print(f"\n=======================================================")
    print(f" [SIH BEL DEMO] Aisle Blockage Injection Event")
    print(f" Target AMR : /{args.robot}")
    print(f" State      : {'>>> AISLE BLOCKED <<<' if is_blocked else 'CLEARED'}")
    print(f" Triggered  : Edge Perception -> D* Re-Route -> Contract Net Auction")
    print(f"=======================================================\n")

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
