#!/usr/bin/env python3
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from mos_sdk import ARM_ID, POSE, MosController

from mos_arm_controller.srv import RunTask


class ArmControllerNode(Node):
    """Arm service node for grasp, place, and elevator tasks."""

    SUPPORTED_TASKS = ("grasp", "place", "elevator")

    def __init__(self):
        super().__init__("mos_arm_controller")

        self._running_lock = threading.Lock()
        self._running = False

        self._controller = MosController()
        self._controller.connect()
        self.get_logger().info("connected MosController")

        self.run_task_service = self.create_service(
            RunTask, "~/run_task", self._handle_run_task
        )
        self.get_logger().info(
            f"ready ~/run_task, supported tasks: {', '.join(self.SUPPORTED_TASKS)}"
        )

    def _handle_run_task(self, request, response):
        task_name = str(request.task_name or "").strip().lower()
        if task_name not in self.SUPPORTED_TASKS:
            response.success = False
            response.message = (
                f"unsupported task: {request.task_name!r}; "
                f"expected one of {self.SUPPORTED_TASKS}"
            )
            self.get_logger().error(response.message)
            return response

        with self._running_lock:
            if self._running:
                response.success = False
                response.message = "another arm task is already running"
                self.get_logger().warn(response.message)
                return response
            self._running = True

        try:
            self.get_logger().info(
                f"start task={task_name} "
                f"left=({request.left_point.point.x:.4f},"
                f"{request.left_point.point.y:.4f},"
                f"{request.left_point.point.z:.4f}) "
                f"right=({request.right_point.point.x:.4f},"
                f"{request.right_point.point.y:.4f},"
                f"{request.right_point.point.z:.4f}) "
                f"frame={request.target_frame}"
            )
            if task_name == "grasp":
                self._do_grasp(request)
            elif task_name == "place":
                self._do_place(request)
            elif task_name == "elevator":
                self._do_elevator(request)
            response.success = True
            response.message = f"task {task_name} finished"
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f"task {task_name} failed: {exc}"
            self.get_logger().error(f"{response.message}\n{traceback.format_exc()}")
        finally:
            with self._running_lock:
                self._running = False

        return response

    def _do_grasp(self, request):
        left_pose = self._build_pose(
            request.left_point,
            rotation=[-0.7071, 0.0, 0.0, 0.7071],
            x_offset=-0.2,
            z_offset=0.02,
        )
        right_pose = self._build_pose(
            request.right_point,
            rotation=[0.7071, 0.0, 0.0, 0.7071],
            x_offset=-0.2,
            z_offset=0.02,
        )

        self.get_logger().info("grasp: go home")
        self._controller.go_home(ARM_ID.BOTH_ARM)
        time.sleep(10)

        self.get_logger().info("grasp: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(5)

        self.get_logger().info("grasp: move left arm")
        self._controller.set_cartesian_planing(ARM_ID.LEFT_ARM, left_pose)
        time.sleep(5)

        self.get_logger().info("grasp: move right arm")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        time.sleep(5)

        self.get_logger().info("grasp: close grippers")
        self._controller.set_left_gripper(1.0)
        self._controller.set_right_gripper(1.0)
        time.sleep(5)

        self.get_logger().info("grasp: go zero")
        self._controller.go_zero(ARM_ID.BOTH_ARM)
        time.sleep(5)

    def _do_place(self, request):
        del request
        self.get_logger().info("place: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(5)

        self.get_logger().info("place: move both arms backward")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.BOTH_ARM, -0.15, 0.0, 0.0
        )
        time.sleep(5)

        self.get_logger().info("place: go zero")
        self._controller.go_zero(ARM_ID.BOTH_ARM)
        time.sleep(5)

    def _do_elevator(self, request):
        right_pose = self._build_pose(
            request.right_point,
            rotation=[0.7071, 0.0, 0.0, 0.7071],
            x_offset=-0.2,
            z_offset=0.02,
        )

        self.get_logger().info("elevator: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(2)

        self.get_logger().info("elevator: lift right arm away from cart")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.RIGHT_ARM, 0.0, 0.0, 0.1
        )
        time.sleep(3)

        self.get_logger().info("elevator: close grippers")
        self._controller.set_left_gripper(1.0)
        self._controller.set_right_gripper(1.0)
        time.sleep(2)

        state = self._controller.get_robot_state()
        self.get_logger().info(
            f"elevator: right_tcp_pose xyz={state.right_tcp_pose.position} "
            f"xyzw={state.right_tcp_pose.rotation}"
        )
        time.sleep(2)

        return_pose = POSE()
        return_pose.position = state.right_tcp_pose.position
        return_pose.rotation = [0.7071, 0.0, 0.0, 0.7071]

        self.get_logger().info("elevator: press button")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        time.sleep(3)

        self.get_logger().info("elevator: return right arm")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, return_pose)
        time.sleep(3)

        self.get_logger().info("elevator: reopen grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(2)

        self.get_logger().info("elevator: lower right arm")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.RIGHT_ARM, 0.0, 0.0, -0.1
        )
        time.sleep(1)

        self.get_logger().info("elevator: close grippers")
        self._controller.set_left_gripper(1.0)
        self._controller.set_right_gripper(1.0)
        time.sleep(2)

    @staticmethod
    def _build_pose(point_stamped, rotation, x_offset=0.0, z_offset=0.0):
        point = point_stamped.point
        pose = POSE()
        pose.position = [
            float(point.x) + x_offset,
            float(point.y),
            float(point.z) + z_offset,
        ]
        pose.rotation = list(rotation)
        return pose

    def destroy_node(self):
        if self._controller is not None:
            try:
                self._controller.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"failed to disconnect MosController: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
