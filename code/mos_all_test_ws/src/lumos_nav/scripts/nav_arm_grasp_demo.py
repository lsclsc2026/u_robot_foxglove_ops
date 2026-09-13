#!/usr/bin/env python3
import argparse
import math


def quaternion_from_yaw(yaw_rad):
    half_yaw = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def fill_home_request(request):
    request.task_name = "home"
    request.left_point.header.frame_id = ""
    request.right_point.header.frame_id = ""
    request.left_point.point.x = 0.0
    request.left_point.point.y = 0.0
    request.left_point.point.z = 0.0
    request.right_point.point.x = 0.0
    request.right_point.point.y = 0.0
    request.right_point.point.z = 0.0
    request.target_frame = ""
    return request


def fill_grasp_request(request, left_xyz, right_xyz, target_frame):
    request.task_name = "grasp"
    request.left_point.header.frame_id = str(target_frame)
    request.right_point.header.frame_id = str(target_frame)
    request.left_point.point.x = float(left_xyz[0])
    request.left_point.point.y = float(left_xyz[1])
    request.left_point.point.z = float(left_xyz[2])
    request.right_point.point.x = float(right_xyz[0])
    request.right_point.point.y = float(right_xyz[1])
    request.right_point.point.z = float(right_xyz[2])
    request.target_frame = str(target_frame)
    return request


def build_goal_pose_message(node, pose_stamped_cls, frame_id, xyz, yaw_rad):
    pose = pose_stamped_cls()
    pose.header.frame_id = str(frame_id)
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(xyz[0])
    pose.pose.position.y = float(xyz[1])
    pose.pose.position.z = float(xyz[2])
    qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Navigate with lumos_nav, then call home and grasp on fused arm service"
    )
    parser.add_argument("--ros-args", nargs="*", help=argparse.SUPPRESS)
    parser.parse_known_args(argv)

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node

    from mos_arm_controller.srv import RunTask

    class NavArmGraspDemoNode(Node):
        def __init__(self):
            super().__init__("nav_arm_grasp_demo")
            self._declare_parameters()
            self._load_parameters()
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self._arm_client = self.create_client(
                RunTask, "/mos_arm_controller/run_task"
            )

        def _declare_parameters(self):
            self.declare_parameter("goal_frame_id", "map")
            self.declare_parameter("goal_x", 1.0)
            self.declare_parameter("goal_y", 0.0)
            self.declare_parameter("goal_z", 0.0)
            self.declare_parameter("goal_yaw_rad", 0.0)
            self.declare_parameter("nav_timeout_sec", 120.0)
            self.declare_parameter("nav_server_wait_sec", 10.0)
            self.declare_parameter("arm_service_wait_sec", 10.0)
            self.declare_parameter("do_home_before_grasp", True)
            self.declare_parameter("grasp_frame_id", "base_link")
            self.declare_parameter("left_grasp_xyz", [0.45, 0.12, 0.18])
            self.declare_parameter("right_grasp_xyz", [0.45, -0.12, 0.18])

        def _load_parameters(self):
            self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
            self.goal_xyz = (
                float(self.get_parameter("goal_x").value),
                float(self.get_parameter("goal_y").value),
                float(self.get_parameter("goal_z").value),
            )
            self.goal_yaw_rad = float(self.get_parameter("goal_yaw_rad").value)
            self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
            self.nav_server_wait_sec = float(
                self.get_parameter("nav_server_wait_sec").value
            )
            self.arm_service_wait_sec = float(
                self.get_parameter("arm_service_wait_sec").value
            )
            self.do_home_before_grasp = bool(
                self.get_parameter("do_home_before_grasp").value
            )
            self.grasp_frame_id = str(self.get_parameter("grasp_frame_id").value)
            self.left_grasp_xyz = tuple(self.get_parameter("left_grasp_xyz").value)
            self.right_grasp_xyz = tuple(self.get_parameter("right_grasp_xyz").value)

        def run(self):
            self._wait_for_nav_server()
            self._wait_for_arm_service()
            self._navigate_to_goal()
            if self.do_home_before_grasp:
                self._call_arm_task(fill_home_request(RunTask.Request()), "home")
            grasp_request = fill_grasp_request(
                RunTask.Request(),
                self.left_grasp_xyz,
                self.right_grasp_xyz,
                self.grasp_frame_id,
            )
            self._call_arm_task(grasp_request, "grasp")
            self.get_logger().info("nav_arm_grasp_demo finished successfully")

        def _wait_for_nav_server(self):
            self.get_logger().info("waiting for navigate_to_pose action server")
            if not self._nav_client.wait_for_server(timeout_sec=self.nav_server_wait_sec):
                raise RuntimeError("navigate_to_pose action server not available")

        def _wait_for_arm_service(self):
            self.get_logger().info("waiting for /mos_arm_controller/run_task service")
            if not self._arm_client.wait_for_service(
                timeout_sec=self.arm_service_wait_sec
            ):
                raise RuntimeError("/mos_arm_controller/run_task service not available")

        def _navigate_to_goal(self):
            goal_pose = build_goal_pose_message(
                self, PoseStamped, self.goal_frame_id, self.goal_xyz, self.goal_yaw_rad
            )
            goal = NavigateToPose.Goal()
            goal.pose = goal_pose
            self.get_logger().info(
                "sending nav goal "
                f"frame={self.goal_frame_id} "
                f"xyz=({self.goal_xyz[0]:.3f}, {self.goal_xyz[1]:.3f}, {self.goal_xyz[2]:.3f}) "
                f"yaw={self.goal_yaw_rad:.3f}"
            )
            send_future = self._nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("navigation goal was rejected")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=self.nav_timeout_sec
            )
            if not result_future.done():
                raise RuntimeError("navigation timed out")

            result = result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"navigation failed with status={result.status}")
            self.get_logger().info("navigation goal reached")

        def _call_arm_task(self, request, label):
            self.get_logger().info(f"calling arm task={label}")
            future = self._arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is None:
                raise RuntimeError(f"{label} task returned no response")
            if not response.success:
                raise RuntimeError(f"{label} task failed: {response.message}")
            self.get_logger().info(f"{label} task finished: {response.message}")

    rclpy.init(args=argv)
    node = NavArmGraspDemoNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
