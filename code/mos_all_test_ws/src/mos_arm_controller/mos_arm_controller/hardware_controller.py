#!/usr/bin/env python3
import time
import traceback

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mos_sdk import ARM_ID, POSE, MosController

from mos_arm_controller.srv import RunTask

# 兼容老环境：mos_chassis_controller 可能不存在
try:
    from mos_chassis_controller.srv import MoveChassis
except ImportError:
    MoveChassis = None


class HardwareControllerNode(Node):
    """Single MosController owner that exposes arm and chassis services."""

    SUPPORTED_TASKS = ("grasp", "place", "elevator")

    def __init__(self):
        super().__init__("mos_hardware_controller")
        self._declare_parameters()
        self._load_parameters()

        self._controller = MosController()
        self._controller.connect()
        self.get_logger().info("connected MosController")

        self.run_task_service = self.create_service(
            RunTask, "/mos_arm_controller/run_task", self._handle_run_task
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self.get_logger().info(
            "ready service: /mos_arm_controller/run_task; subscribed /cmd_vel"
        )

    def _declare_parameters(self):
        self.declare_parameter("chassis_threshold_x_m", 0.8579)
        self.declare_parameter("chassis_speed_mps", 0.05)
        self.declare_parameter("grasp_x_offset_m", -0.19)
        self.declare_parameter("grasp_z_offset_m", 0.02)
        self.declare_parameter("home_wait_sec", 3.0)
        self.declare_parameter("arm_motion_wait_sec", 5.0)
        self.declare_parameter("gripper_wait_sec", 5.0)
        self.declare_parameter("elevator_short_wait_sec", 2.0)
        self.declare_parameter("elevator_motion_wait_sec", 3.0)
        self.declare_parameter("return_arms_home_xy_after_grasp", True)

    def _load_parameters(self):
        self.chassis_threshold_x_m = float(
            self.get_parameter("chassis_threshold_x_m").value
        )
        self.chassis_speed_mps = float(self.get_parameter("chassis_speed_mps").value)
        self.grasp_x_offset_m = float(self.get_parameter("grasp_x_offset_m").value)
        self.grasp_z_offset_m = float(self.get_parameter("grasp_z_offset_m").value)
        self.home_wait_sec = float(self.get_parameter("home_wait_sec").value)
        self.arm_motion_wait_sec = float(
            self.get_parameter("arm_motion_wait_sec").value
        )
        self.gripper_wait_sec = float(self.get_parameter("gripper_wait_sec").value)
        self.elevator_short_wait_sec = float(
            self.get_parameter("elevator_short_wait_sec").value
        )
        self.elevator_motion_wait_sec = float(
            self.get_parameter("elevator_motion_wait_sec").value
        )
        self.return_arms_home_xy_after_grasp = bool(
            self.get_parameter("return_arms_home_xy_after_grasp").value
        )
        if self.chassis_speed_mps <= 0.0:
            raise ValueError("chassis_speed_mps must be positive")

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

        try:
            self.get_logger().info(
                f"start arm task={task_name} "
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
            self.get_logger().error(response.message)
            self.get_logger().error(traceback.format_exc())

        return response

    def _on_cmd_vel(self, msg: Twist):
        """Forward /cmd_vel to chassis SDK, replacing mos_slam_diff_drive_controller."""
        try:
            self._controller.set_chassis_velocity(
                float(msg.linear.x), 0.0, float(msg.angular.z)
            )
        except Exception as exc:
            self.get_logger().warn(f"cmd_vel forward failed: {exc}")

    def _move_chassis_to_grasp_position(self, request):
        """根据把手 x 距离移动底盘到抓取位置，返回 (moved_distance, duration)。

        - target_x > threshold: 前进 (target_x - threshold)
        - target_x < threshold: 后退 (threshold - target_x)
        - target_x ≈ threshold: 不移动

        moved_distance 为带符号值：正=前进，负=后退，0=未移动。
        """
        target_x = (
            float(request.left_point.point.x)
            + float(request.right_point.point.x)
        ) / 2.0
        offset_x = target_x - self.chassis_threshold_x_m

        if abs(offset_x) < 1e-4:
            self.get_logger().info(
                f"chassis skip: target_x={target_x:.4f} within threshold "
                f"{self.chassis_threshold_x_m:.4f}"
            )
            return 0.0, 0.0

        speed = self.chassis_speed_mps if offset_x > 0 else -self.chassis_speed_mps
        duration = abs(offset_x) / self.chassis_speed_mps
        action = "forward" if offset_x > 0 else "backward"
        self.get_logger().info(
            f"chassis {action}: target_x={target_x:.4f}, offset={offset_x:.3f} m, "
            f"speed={speed:.3f} m/s, duration={duration:.2f} s"
        )
        self._controller.set_chassis_velocity(speed, 0.0, 0.0)
        time.sleep(duration)
        self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
        return offset_x, duration

    def _return_chassis_to_origin(self, moved_distance: float, duration: float):
        """反向移动底盘回到原位（与 moved_distance 等距反向）。"""
        if abs(moved_distance) < 1e-4:
            return
        speed = -self.chassis_speed_mps if moved_distance > 0 else self.chassis_speed_mps
        action = "backward" if moved_distance > 0 else "forward"
        self.get_logger().info(
            f"chassis return {action}: distance={abs(moved_distance):.3f} m, "
            f"speed={speed:.3f} m/s, duration={duration:.2f} s"
        )
        self._controller.set_chassis_velocity(speed, 0.0, 0.0)
        time.sleep(duration)
        self._controller.set_chassis_velocity(0.0, 0.0, 0.0)

    def _do_grasp(self, request):
        # ===== 1. 底盘移动到抓取位置 =====
        # 底盘按左右把手 x 平均值移动，移动后视觉坐标平均为 threshold=0.8579
        # 必须先移动底盘，避免太近时 go_home 撞到小车
        moved_distance, duration = self._move_chassis_to_grasp_position(request)

        # ===== 2. 机械臂回 home 点（底盘到位后再回 home，安全）=====
        self.get_logger().info("grasp: go home")
        self._controller.go_home(ARM_ID.BOTH_ARM)
        time.sleep(self.home_wait_sec)
        # 记录 home 点的 xy，抓取后用于"回 home xy (保持 z)"动作
        left_home_xy, right_home_xy = self._read_home_xy()

        # ===== 3. 构造抓取位姿（每个把手用自己的 x 补偿）=====
        # 底盘移动 moved_distance 后，把手在视觉坐标系下的新 x = point.x - moved_distance
        # 转机械臂坐标系：x_arm = (point.x - moved_distance) + grasp_x_offset_m
        #                                └────────────────────┘   └────────────┘
        #                                底盘移动后视觉坐标        视觉→机械臂偏移(-0.19)
        left_x_arm = float(request.left_point.point.x) - moved_distance + self.grasp_x_offset_m
        right_x_arm = float(request.right_point.point.x) - moved_distance + self.grasp_x_offset_m
        left_pose = self._build_pose_fixed_x(
            request.left_point,
            target_x=left_x_arm,
            rotation=[-0.7071, 0.0, 0.0, 0.7071],
            z_offset=self.grasp_z_offset_m,
        )
        right_pose = self._build_pose_fixed_x(
            request.right_point,
            target_x=right_x_arm,
            rotation=[0.7071, 0.0, 0.0, 0.7071],
            z_offset=self.grasp_z_offset_m,
        )

        # ===== 4. 执行抓取 =====
        self.get_logger().info("grasp: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(self.gripper_wait_sec)

        self.get_logger().info("grasp: move left arm")
        self._controller.set_cartesian_planing(ARM_ID.LEFT_ARM, left_pose)
        time.sleep(self.arm_motion_wait_sec)

        self.get_logger().info("grasp: move right arm")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        time.sleep(self.arm_motion_wait_sec)

        self.get_logger().info("grasp: close grippers")
        self._controller.set_left_gripper(0.83)
        self._controller.set_right_gripper(0.83)
        time.sleep(self.gripper_wait_sec)

        # ===== 5. 机械臂回到 home 的 xy（保持当前 z 和姿态）=====
        # 替代原"双臂后退 10cm"：让机械臂水平回收至 home xy，避免划过身体
        if self.return_arms_home_xy_after_grasp:
            self._return_arms_home_xy_keep_z(left_home_xy, right_home_xy)

        # ===== 6. 底盘反向移动回原位 =====
        self._return_chassis_to_origin(moved_distance, duration)


    def _do_place(self, request):
        del request
        self.get_logger().info("place: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(self.gripper_wait_sec)

        self.get_logger().info("place: move both arms backward")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.BOTH_ARM, -0.15, 0.0, 0.0
        )
        time.sleep(self.arm_motion_wait_sec)

        self.get_logger().info("place: go zero")
        self._controller.go_zero(ARM_ID.BOTH_ARM)
        time.sleep(self.arm_motion_wait_sec)

        left_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5]
        right_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5]

        self._controller.set_angle_J(ARM_ID.LEFT_ARM, left_joint_angles)
        time.sleep(2)

        self._controller.set_angle_J(ARM_ID.RIGHT_ARM, right_joint_angles)
        time.sleep(2)


    def _do_elevator(self, request):
        right_pose = self._build_pose(
            request.right_point,
            rotation=[0.7071, 0.0, 0.0, 0.7071],
            x_offset=self.grasp_x_offset_m,
            z_offset=self.grasp_z_offset_m,
        )

        self.get_logger().info("elevator: open grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(self.elevator_short_wait_sec)

        self.get_logger().info("elevator: lift right arm away from cart")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.RIGHT_ARM, 0.0, 0.0, 0.1
        )
        time.sleep(self.elevator_motion_wait_sec)

        self.get_logger().info("elevator: close grippers")
        self._controller.set_left_gripper(1.0)
        self._controller.set_right_gripper(1.0)
        time.sleep(self.elevator_short_wait_sec)

        state = self._controller.get_robot_state()
        self.get_logger().info(
            f"elevator: right_tcp_pose xyz={state.right_tcp_pose.position} "
            f"xyzw={state.right_tcp_pose.rotation}"
        )
        time.sleep(self.elevator_short_wait_sec)

        return_pose = POSE()
        return_pose.position = state.right_tcp_pose.position
        return_pose.rotation = [0.7071, 0.0, 0.0, 0.7071]

        self.get_logger().info("elevator: press button")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        time.sleep(self.elevator_motion_wait_sec)

        self.get_logger().info("elevator: return right arm")
        self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, return_pose)
        time.sleep(self.elevator_motion_wait_sec)

        self.get_logger().info("elevator: reopen grippers")
        self._controller.set_left_gripper(0.0)
        self._controller.set_right_gripper(0.0)
        time.sleep(self.elevator_short_wait_sec)

        self.get_logger().info("elevator: lower right arm")
        self._controller.set_cartesian_position_offset_planing(
            ARM_ID.RIGHT_ARM, 0.0, 0.0, -0.1
        )
        time.sleep(1)

        self.get_logger().info("elevator: close grippers")
        self._controller.set_left_gripper(1.0)
        self._controller.set_right_gripper(1.0)
        time.sleep(self.elevator_short_wait_sec)

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

    @staticmethod
    def _build_pose_fixed_x(point_stamped, target_x, rotation, z_offset=0.0):
        """构造抓取位姿：x 用传入的目标值，y/z 用视觉检测值。

        用于底盘已移动到 threshold 后的抓取场景：
        - x = target_x（机械臂坐标系下的目标 x，已包含坐标系转换）
        - y = point.y（左右手把 y 不变）
        - z = point.z + z_offset
        """
        point = point_stamped.point
        pose = POSE()
        pose.position = [
            float(target_x),
            float(point.y),
            float(point.z) + z_offset,
        ]
        pose.rotation = list(rotation)
        return pose

    def _read_home_xy(self):
        """读取当前双臂 tcp 的 xy 坐标（用于 go_home 后记录 home 位置）。

        返回 ((left_x, left_y), (right_x, right_y))。
        """
        state = self._controller.get_robot_state()
        left_xy = (
            float(state.left_tcp_pose.position[0]),
            float(state.left_tcp_pose.position[1]),
        )
        right_xy = (
            float(state.right_tcp_pose.position[0]),
            float(state.right_tcp_pose.position[1]),
        )
        self.get_logger().info(
            f"home xy: left={left_xy}, right={right_xy}"
        )
        return left_xy, right_xy

    def _return_arms_home_xy_keep_z(self, left_home_xy, right_home_xy):
        """让双臂回到 home 的 xy，但保持当前 z 高度和姿态。

        用于抓取完成后回收机械臂：水平移动到 home 的 xy 位置，避免
        直接 go_home 导致 z 方向突变或划过身体。
        """
        state = self._controller.get_robot_state()
        left_x = float(state.left_tcp_pose.position[0])
        right_x = float(state.right_tcp_pose.position[0])
        left_y = float(state.left_tcp_pose.position[1])
        right_y = float(state.right_tcp_pose.position[1])

        move_left_x = left_home_xy[0] - left_x
        move_right_x = right_home_xy[0] - right_x
        move_left_y = left_home_xy[1] - left_y
        move_right_y = right_home_xy[1] - right_y

        move_x = (move_left_x + move_right_x) / 2
        move_y = (move_left_y + move_right_y) / 2

        self.get_logger().info(
            f"return arms home xy: left=({move_left_x:.3f},{move_left_y:.3f}), "
            f"right=({move_right_x:.3f},{move_right_y:.3f}), "
            f"avg=({move_x:.3f},{move_y:.3f})"
        )

        # 分步移动：先 Y 后 X，减少 IK 解空间，降低肘部外翻概率
        if abs(move_y) > 1e-4:
            self.get_logger().info(f"step 1: move Y only, dy={move_y:.3f}")
            self._controller.set_cartesian_position_offset_planing(
                ARM_ID.BOTH_ARM, 0.0, move_y, 0.0
            )
            time.sleep(self.arm_motion_wait_sec)

        if abs(move_x) > 1e-4:
            self.get_logger().info(f"step 2: move X only, dx={move_x:.3f}")
            self._controller.set_cartesian_position_offset_planing(
                ARM_ID.BOTH_ARM, move_x, 0.0, 0.0
            )
            time.sleep(self.arm_motion_wait_sec)

    def destroy_node(self):
        if self._controller is not None:
            try:
                self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
            except Exception as exc:
                self.get_logger().warn(f"failed to stop chassis safely: {exc}")
            try:
                self._controller.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"failed to disconnect MosController: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


