#!/usr/bin/env python3
import json
import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler


def main():
    if len(sys.argv) < 2:
        print(f"用法: cruise_waypoints.py <waypoints.json>")
        print(f"JSON 格式: {{'waypoints':[{{'name':'门口','x':1,'y':2,'yaw':0}}, ...]}}")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        data = json.load(f)

    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    names = []
    waypoints = []
    for wp in data["waypoints"]:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = wp["x"]
        p.pose.position.y = wp["y"]
        q = quaternion_from_euler(0, 0, wp["yaw"])
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        waypoints.append(p)
        names.append(wp.get("name", f"waypoint_{len(names)}"))

    print(f"共 {len(waypoints)} 个点:", " → ".join(names))
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            idx = fb.current_waypoint
            name = names[idx] if idx < len(names) else str(idx)
            print(f"\r  [{name}] ({idx+1}/{len(waypoints)})", end="")

    print(f"\n巡航完成: {nav.getResult()}")


if __name__ == "__main__":
    main()
