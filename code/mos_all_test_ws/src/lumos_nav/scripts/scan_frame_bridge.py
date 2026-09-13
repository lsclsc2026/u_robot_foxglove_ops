#!/usr/bin/env python3
"""Republish /scan as /scan_nav2 with sensor_data QoS for nav2 obstacle layer."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameBridge(Node):
    def __init__(self):
        super().__init__('scan_frame_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(LaserScan, '/scan_nav2', qos)
        self._subscription = self.create_subscription(
            LaserScan, '/scan', self._callback, qos)
        self.get_logger().info(
            'scan_frame_bridge started: /scan -> /scan_nav2(livox_frame, now())')

    def _callback(self, msg):
        msg.header.frame_id = 'livox_frame'
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
