#!/usr/bin/env python3
"""Republish cloud_registered with current system timestamps for Nav2 consumers."""

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CloudStampBridge(Node):
    def __init__(self):
        super().__init__('cloud_stamp_bridge')
        self._publisher = self.create_publisher(PointCloud2, '/cloud_registered_nav2', 10)
        self._subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self._callback,
            10,
        )
        self.get_logger().info(
            'cloud_stamp_bridge started (republishing /cloud_registered -> /cloud_registered_nav2 with current timestamps)'
        )

    def _callback(self, msg):
        stamped = copy.deepcopy(msg)
        stamped.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(stamped)


def main():
    rclpy.init()
    node = CloudStampBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
