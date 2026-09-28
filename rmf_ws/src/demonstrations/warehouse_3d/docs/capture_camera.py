#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class CameraSnapshot(Node):
    def __init__(self, topic, output_file):
        super().__init__('camera_snapshot')
        self.output_file = output_file
        self.sub = self.create_subscription(Image, topic, self.image_callback, 10)
        self.captured = False
        self.get_logger().info(f"Waiting for frame on {topic}...")

    def image_callback(self, msg: Image):
        if self.captured:
            return
        try:
            # Assume rgb8 or bgr8
            if msg.encoding in ['rgb8', 'RGB8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ['bgr8', 'BGR8']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            elif msg.encoding in ['mono8', '8UC1']:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            
            cv2.imwrite(self.output_file, img)
            self.get_logger().info(f"Snapshot saved to {self.output_file}")
            self.captured = True
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/robot_alpha/camera/image_raw'
    output_file = sys.argv[2] if len(sys.argv) > 2 else '/tmp/camera_snapshot.png'
    rclpy.init()
    node = CameraSnapshot(topic, output_file)
    timeout = 15.0
    start = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok() and not node.captured:
        rclpy.spin_once(node, timeout_sec=0.5)
        now = node.get_clock().now().nanoseconds / 1e9
        if now - start > timeout:
            node.get_logger().warn("Snapshot timeout reached.")
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
