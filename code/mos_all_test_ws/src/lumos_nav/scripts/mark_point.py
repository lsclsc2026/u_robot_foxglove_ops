#!/usr/bin/env python3
"""
手动走点到目标位置后运行此脚本，自动记录当前位姿到 waypoints.json。

用法:
  - 新建文件:    ros2 run lumos_nav mark_point.py -n 门口 -f waypoints.json
  - 追加到已有:  ros2 run lumos_nav mark_point.py -n 充电桩 -f waypoints.json -a
  - 不命名:      ros2 run lumos_nav mark_point.py -f waypoints.json -a   (自动编号)
  - 打印不保存:  ros2 run lumos_nav mark_point.py -p
"""
import argparse
import json
import os
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class PoseMarker(Node):
    def __init__(self):
        super().__init__("pose_marker")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def get_pose(self, timeout=2.0, base_frame="base_footprint", map_frame="map"):
        for _ in range(int(timeout * 10)):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                break
        t = self._tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)}


def main():
    parser = argparse.ArgumentParser(description="标记当前位姿为巡航点")
    parser.add_argument("-f", "--file", default="waypoints.json", help="输出文件")
    parser.add_argument("-n", "--name", default=None, help="自定义点位名称")
    parser.add_argument("-a", "--append", action="store_true", help="追加模式（否则覆盖）")
    parser.add_argument("-p", "--print-only", action="store_true", help="只打印不保存")
    args = parser.parse_args()

    rclpy.init()
    marker = PoseMarker()
    pose = marker.get_pose()
    rclpy.shutdown()

    if args.print_only:
        print(f"  当前位姿: x={pose['x']}, y={pose['y']}, yaw={pose['yaw']}")
        return

    data = {"waypoints": []}
    if args.append and os.path.exists(args.file):
        with open(args.file) as f:
            data = json.load(f)

    name = args.name or f"waypoint_{len(data['waypoints']) + 1}"
    pose["name"] = name

    data["waypoints"].append(pose)

    with open(args.file, "w") as f:
        json.dump(data, f, indent=2)

    print(f"  标定点 [{name}]: x={pose['x']}, y={pose['y']}, yaw={pose['yaw']}")
    print(f"  → 已写入 {args.file} (共 {len(data['waypoints'])} 个点)")


if __name__ == "__main__":
    main()
