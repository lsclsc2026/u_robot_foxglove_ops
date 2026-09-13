#!/usr/bin/env python3
"""Publish a stable arrival event when the robot reaches the latest goal pose."""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GoalArrivalMonitor(Node):
    def __init__(self):
        super().__init__('goal_arrival_monitor')
        self._declare_parameters()
        self._load_parameters()

        self._goal_msg = None
        self._goal_id = ''
        self._arrival_published = False
        self._within_tolerance_since = None

        self._sub_goal = self.create_subscription(
            PoseStamped, self.goal_topic, self._on_goal, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 20
        )
        self._pub_arrival = self.create_publisher(String, self.arrival_topic, 10)

        self.get_logger().info(
            'goal_arrival_monitor started '
            f'goal_topic={self.goal_topic} odom_topic={self.odom_topic} '
            f'arrival_topic={self.arrival_topic}'
        )

    def _declare_parameters(self):
        defaults = {
            'goal_topic': '/goal_pose',
            'odom_topic': '/localization',
            'arrival_topic': '/mosnav/arrival',
            'xy_tolerance': 0.25,
            'yaw_tolerance': 0.25,
            'hold_time_sec': 0.5,
            'max_linear_speed': 0.05,
            'max_angular_speed': 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.arrival_topic = str(self.get_parameter('arrival_topic').value)
        self.xy_tolerance = max(0.0, float(self.get_parameter('xy_tolerance').value))
        self.yaw_tolerance = max(0.0, float(self.get_parameter('yaw_tolerance').value))
        self.hold_time_sec = max(0.0, float(self.get_parameter('hold_time_sec').value))
        self.max_linear_speed = max(
            0.0, float(self.get_parameter('max_linear_speed').value)
        )
        self.max_angular_speed = max(
            0.0, float(self.get_parameter('max_angular_speed').value)
        )

    def _on_goal(self, msg):
        self._goal_msg = msg
        self._goal_id = self._build_goal_id(msg)
        self._arrival_published = False
        self._within_tolerance_since = None

        position = msg.pose.position
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.get_logger().info(
            f'received goal goal_id={self._goal_id} frame={msg.header.frame_id} '
            f'x={position.x:.3f} y={position.y:.3f} yaw={yaw:.3f}'
        )

    def _on_odom(self, msg):
        if self._goal_msg is None or self._arrival_published:
            return

        goal_position = self._goal_msg.pose.position
        goal_yaw = yaw_from_quaternion(self._goal_msg.pose.orientation)
        current_position = msg.pose.pose.position
        current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        dx = float(current_position.x) - float(goal_position.x)
        dy = float(current_position.y) - float(goal_position.y)
        distance_xy = math.hypot(dx, dy)
        yaw_error = abs(normalize_angle(current_yaw - goal_yaw))

        linear_speed = math.hypot(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        angular_speed = abs(float(msg.twist.twist.angular.z))

        within_tolerance = (
            distance_xy <= self.xy_tolerance
            and yaw_error <= self.yaw_tolerance
            and linear_speed <= self.max_linear_speed
            and angular_speed <= self.max_angular_speed
        )

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if within_tolerance:
            if self._within_tolerance_since is None:
                self._within_tolerance_since = now_sec
                return
            if now_sec - self._within_tolerance_since < self.hold_time_sec:
                return
            self._publish_arrival(distance_xy, yaw_error, current_position, current_yaw)
            return

        self._within_tolerance_since = None

    def _publish_arrival(self, distance_xy, yaw_error, current_position, current_yaw):
        goal_position = self._goal_msg.pose.position
        goal_yaw = yaw_from_quaternion(self._goal_msg.pose.orientation)
        payload = {
            'goal_id': self._goal_id,
            'status': 'reached',
            'goal_frame': str(self._goal_msg.header.frame_id or ''),
            'odom_frame': str(self._goal_msg.header.frame_id or ''),
            'goal_pose': {
                'x': float(goal_position.x),
                'y': float(goal_position.y),
                'yaw': float(goal_yaw),
            },
            'current_pose': {
                'x': float(current_position.x),
                'y': float(current_position.y),
                'yaw': float(current_yaw),
            },
            'distance_xy': float(distance_xy),
            'yaw_error': float(yaw_error),
            'timestamp_ns': int(self.get_clock().now().nanoseconds),
        }
        self._pub_arrival.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self._arrival_published = True
        self._within_tolerance_since = None
        self.get_logger().info(
            f'published arrival goal_id={self._goal_id} '
            f'distance_xy={distance_xy:.3f} yaw_error={yaw_error:.3f}'
        )

    @staticmethod
    def _build_goal_id(msg):
        stamp = msg.header.stamp
        position = msg.pose.position
        yaw = yaw_from_quaternion(msg.pose.orientation)
        return (
            f'{int(stamp.sec)}.{int(stamp.nanosec)}:'
            f'{float(position.x):.3f}:{float(position.y):.3f}:{yaw:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalArrivalMonitor()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass
