#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对标 mos_slam_get_map_pose -> get_pose.py 的 Nav2 版本。
订阅 odom 话题，采集 N 秒数据，取均值后写入 YAML 标定点文件。

用法:
  ros2 run lumos_nav nav2_get_pose.py -t a
  ros2 run lumos_nav nav2_get_pose.py -t b -n 充电桩
  ros2 run lumos_nav nav2_get_pose.py -t c -d 5.0 -f ~/my_waypoints.yaml
"""
import argparse
import os
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaypointRecorder(Node):
    def __init__(self, target_key, name, odom_topic, duration, file_path):
        super().__init__("nav2_waypoint_recorder")
        self.target_key = target_key
        self.custom_name = name
        self.duration = duration
        self.file_path = file_path

        self.data_list = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.is_recording = True

        self.sub = self.create_subscription(Odometry, odom_topic, self._cb, 10)
        self.get_logger().info(
            f"开始采集 [{target_key}] odom={odom_topic} 时长={duration}s"
        )

    def _cb(self, msg):
        if not self.is_recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.start_time > self.duration:
            self.is_recording = False
            self._write_file()
            raise SystemExit
        self.data_list.append(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )

    def _write_file(self):
        N = len(self.data_list)
        if N == 0:
            self.get_logger().error("采集失败: 无数据")
            return

        data = np.array(self.data_list)
        pos_mean = np.mean(data[:, :3], axis=0)
        q_sum = np.sum(data[:, 3:7], axis=0)
        norm = np.linalg.norm(q_sum)
        q_mean = q_sum / norm if norm > 0 else q_sum
        std_pos = np.std(data[:, :3], axis=0, ddof=1)
        std_xy = np.sqrt(std_pos[0] ** 2 + std_pos[1] ** 2)

        px, py, pz = round(pos_mean[0], 4), round(pos_mean[1], 4), round(pos_mean[2], 4)
        qw, qx, qy, qz = round(q_mean[0], 4), round(q_mean[1], 4), round(q_mean[2], 4), round(q_mean[3], 4)

        print("\n" + "-" * 40)
        print("  稳定性报告 (N={})".format(N))
        print("-" * 40)
        print(f"  X 抖动: {std_pos[0]:.5f} m,  Y 抖动: {std_pos[1]:.5f} m")
        print(f"  平面综合抖动: {std_xy:.5f} m")
        if std_xy > 0.05:
            self.get_logger().warn("!!! 数据波动 > 5cm，机器人可能未静止，建议重新采集 !!!")
        else:
            print("  >>> 状态: 稳定")
        print("-" * 40 + "\n")

        content = {}
        if os.path.exists(self.file_path):
            with open(self.file_path) as f:
                content = yaml.safe_load(f) or {}

        content[self.target_key] = {
            "name": self.custom_name or self.target_key,
            "position": {"x": float(px), "y": float(py), "z": float(pz)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        }

        with open(self.file_path, "w") as f:
            yaml.safe_dump(content, f, default_flow_style=False, sort_keys=False)

        G = "\033[92m"
        R = "\033[0m"
        print(f"{G}==================================================")
        print(f"  成功！标定点 [{self.target_key}] 已写入")
        print(f"  文件: {self.file_path}")
        print(f"  Position:    x={px}, y={py}, z={pz}")
        print(f"  Orientation: x={qx}, y={qy}, z={qz}, w={qw}")
        print(f"=================================================={R}\n")


def main():
    parser = argparse.ArgumentParser(description="Nav2 标定点位采集")
    parser.add_argument("-t", "--target", required=True, help="点位 key，如 a/b/c/o")
    parser.add_argument("-n", "--name", default=None, help="点位描述名称，如 门口/充电桩")
    parser.add_argument("-d", "--duration", type=float, default=3.0, help="采集时长秒 (default=3)")
    parser.add_argument("-f", "--file", default="nav2_waypoints.yaml", help="输出 YAML (default=nav2_waypoints.yaml)")
    parser.add_argument("--odom", default="/localization", help="odom 话题 (default=/localization)")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = WaypointRecorder(args.target, args.name, args.odom, args.duration, args.file)
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
