#!/usr/bin/env python3
"""Block bringup until the first /map_to_odom message arrives."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaitForMapToOdom(Node):
    def __init__(self):
        super().__init__('wait_for_map_to_odom')
        self._received = False
        self._max_wait_sec = 15.0
        self._elapsed = 0.0
        self._subscription = self.create_subscription(
            Odometry,
            '/map_to_odom',
            self._callback,
            10,
        )
        self._timer = self.create_timer(1.0, self._timeout_check)
        self.get_logger().info(
            f'waiting up to {self._max_wait_sec}s for first /map_to_odom message before starting Nav2')

    def _callback(self, msg):
        if self._received:
            return
        self._received = True
        self.get_logger().info(
            f'received /map_to_odom ({msg.header.frame_id} -> {msg.child_frame_id}), bringup gate opened'
        )
        self.destroy_subscription(self._subscription)
        self._subscription = None
        self.create_timer(0.1, self._gate_opened)

    def _timeout_check(self):
        if self._received:
            return
        self._elapsed += 1.0
        remaining = self._max_wait_sec - self._elapsed
        if remaining <= 0.0:
            self.get_logger().warn(
                f'/map_to_odom not received within {self._max_wait_sec}s, opening gate anyway')
            self._gate_opened()
        else:
            self.get_logger().info(
                f'still waiting for /map_to_odom ({remaining:.0f}s remaining)')

    def _gate_opened(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        raise SystemExit(0)


def main():
    rclpy.init()
    node = WaitForMapToOdom()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
