#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对标 mos_slam_send_goals -> set_goals.py 的 Nav2 版本。
从 YAML 读取标定点，通过 Nav2 NavigateToPose action 逐个导航。

用法:
  ros2 run lumos_nav nav2_send_goals.py a b c
  ros2 run lumos_nav nav2_send_goals.py o a o b
  ros2 run lumos_nav nav2_send_goals.py a b -f ~/my_waypoints.yaml
  ros2 run lumos_nav nav2_send_goals.py --list   (查看可用点位)
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2GoalSender(Node):
    def __init__(self, file_path):
        super().__init__("nav2_goal_sender")
        self._ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.points = {}
        self._load(file_path)

    def _load(self, file_path):
        try:
            with open(file_path) as f:
                self.points = yaml.safe_load(f) or {}
            self.get_logger().info(f"已加载 {len(self.points)} 个点: {list(self.points.keys())}")
        except Exception as e:
            self.get_logger().error(f"加载 YAML 失败: {file_path}: {e}")
            self.points = {}

    def _to_pose(self, point_data):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point_data["position"]["x"])
        pose.pose.position.y = float(point_data["position"]["y"])
        pose.pose.position.z = float(point_data["position"]["z"])
        pose.pose.orientation.x = float(point_data["orientation"]["x"])
        pose.pose.orientation.y = float(point_data["orientation"]["y"])
        pose.pose.orientation.z = float(point_data["orientation"]["z"])
        pose.pose.orientation.w = float(point_data["orientation"]["w"])
        return pose

    def _yaw_deg(self, pose):
        q = pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy) * 180.0 / math.pi

    def _wait_for_server(self):
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server 未就绪！")
            self.get_logger().error("请确认: ros2 launch nav2_bringup navigation_launch.py 已启动")
            return False
        return True

    def send_and_wait(self, goal_pose, timeout=120.0):
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self._ac.send_goal_async(goal).add_done_callback(
            lambda f: self._on_goal_response(f, timeout)
        )

        start = time.monotonic()
        self._goal_reached = False
        self._goal_aborted = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._goal_reached:
                return True
            if self._goal_aborted:
                return False
            if time.monotonic() - start > timeout:
                self.get_logger().warn(f"超时 (>{timeout}s)")
                return False
        return False

    def _on_goal_response(self, future, timeout):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("目标被拒绝")
            self._goal_aborted = True
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._goal_reached = True
        else:
            self.get_logger().warn(f"导航失败 status={result.status}")
            self._goal_aborted = True

    def list_points(self):
        if not self.points:
            print("(无可用点位)")
            return
        print(f"\n可用点位 ({len(self.points)} 个):")
        print("-" * 50)
        for k, v in self.points.items():
            px = v["position"]["x"]
            py = v["position"]["y"]
            name = v.get("name", "")
            print(f"  {k:6s}  {name:12s}  ({px:.2f}, {py:.2f})")
        print("-" * 50)
        print("用法: ros2 run lumos_nav nav2_send_goals.py <点1> <点2> ...\n")


def main():
    parser = argparse.ArgumentParser(description="Nav2 多点导航执行")
    parser.add_argument("waypoints", nargs="*", help="目标点序列，如 a b c")
    parser.add_argument("-f", "--file", default="nav2_waypoints.yaml", help="点位 YAML 文件")
    parser.add_argument("--list", action="store_true", help="列出所有可用点位")
    parser.add_argument("--timeout", type=float, default=500.0, help="单点超时秒")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    sender = Nav2GoalSender(args.file)

    if args.list:
        sender.list_points()
        sender.destroy_node()
        rclpy.shutdown()
        return

    if not args.waypoints:
        sender.list_points()
        sender.destroy_node()
        rclpy.shutdown()
        return

    if not sender._wait_for_server():
        sender.destroy_node()
        rclpy.shutdown()
        return

    # 验证点
    invalid = [p for p in args.waypoints if p not in sender.points]
    if invalid:
        sender.get_logger().error(f"无效点位: {invalid}")
        sender.list_points()
        sender.destroy_node()
        rclpy.shutdown()
        return

    print(f"\n{'='*60}")
    print(f"  航点序列: {' -> '.join(args.waypoints)}")
    print(f"{'='*60}\n")

    for i, key in enumerate(args.waypoints):
        name = sender.points[key].get("name", key)
        pose = sender._to_pose(sender.points[key])
        yaw = sender._yaw_deg(pose)
        sender.get_logger().info(
            f"[{i+1}/{len(args.waypoints)}] 前往 [{name}] ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {yaw:.1f}°)"
        )
        ok = sender.send_and_wait(pose, timeout=args.timeout)
        if not ok:
            sender.get_logger().error(f"[{name}] 导航失败，终止")
            break
        sender.get_logger().info(f"[{name}] 到达")

    sender.get_logger().info(f"\n{'='*60}")
    sender.get_logger().info("  航点序列完成")
    sender.get_logger().info(f"{'='*60}")
    sender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
