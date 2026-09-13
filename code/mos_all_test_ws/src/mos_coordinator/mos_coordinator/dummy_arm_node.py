#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
# from mos_sdk import MosController, ARM_ID, POSE  # SDK 已注释，本地测试不需要

from mos_coordinator.srv import RunTask


class DummyArmNode(Node):
    """机械臂服务节点（DUMMY 版本）。

    对外暴露 `~/run_task` 服务，由 mos_grasp_pipeline 调度器下发任务名与视觉目标点，
    根据任务名分派到 grasp / place / elevator 三种场景动作。
    
    与原始 ww_test_arm.py 的唯一区别：
    - 所有 self._controller.xxx() 调用已注释并替换为 print 语句
    """

    SUPPORTED_TASKS = ("grasp", "place", "elevator")

    def __init__(self):
        super().__init__("dummy_arm_node")

        # 互斥：一次只允许一个任务在执行
        self._running_lock = threading.Lock()
        self._running = False

        # SDK 控制器在节点启动时连接一次
        # self._controller = MosController()
        # self._controller.connect()
        # 替代：模拟连接
        self._controller = None
        self.get_logger().info("【DUMMY】已模拟连接 MosController")

        self.run_task_service = self.create_service(
            RunTask, "~/run_task", self._handle_run_task
        )
        self.get_logger().info(
            f"已提供服务 ~/run_task，支持任务: {', '.join(self.SUPPORTED_TASKS)}"
        )

    def _handle_run_task(self, request, response):
        task_name = str(request.task_name or "").strip().lower()
        if task_name not in self.SUPPORTED_TASKS:
            response.success = False
            response.message = f"不支持的任务名: {request.task_name!r}，应为 {self.SUPPORTED_TASKS}"
            self.get_logger().error(response.message)
            return response

        with self._running_lock:
            if self._running:
                response.success = False
                response.message = "已有任务正在执行，拒绝并发请求"
                self.get_logger().warn(response.message)
                return response
            self._running = True

        try:
            self.get_logger().info(
                f"开始执行任务: {task_name} "
                f"left=({request.left_point.point.x:.4f},{request.left_point.point.y:.4f},{request.left_point.point.z:.4f}) "
                f"right=({request.right_point.point.x:.4f},{request.right_point.point.y:.4f},{request.right_point.point.z:.4f}) "
                f"frame={request.target_frame}"
            )
            if task_name == "grasp":
                self._do_grasp(request)
            elif task_name == "place":
                self._do_place(request)
            elif task_name == "elevator":
                self._do_elevator(request)
            response.success = True
            response.message = f"任务 {task_name} 执行完成"
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f"任务 {task_name} 执行失败: {exc}"
            self.get_logger().error(response.message, exc_info=True)
        finally:
            with self._running_lock:
                self._running = False

        return response

    # ------------------------------------------------------------------
    # 场景动作：与原 ww_test_arm.py 中三个 if 分支一一对应
    # ------------------------------------------------------------------
    def _do_grasp(self, request):
        """抓拿：双臂到目标点抓取。"""
        left_pose = self._build_pose(request.left_point, rotation=[-0.7071, 0.0, 0.0, 0.7071],
                                     x_offset=-0.2, z_offset=0.02)
        right_pose = self._build_pose(request.right_point, rotation=[0.7071, 0.0, 0.0, 0.7071],
                                      x_offset=-0.2, z_offset=0.02)

        self.get_logger().info("grasp: HOME")
        # self._controller.go_home(ARM_ID.BOTH_ARM)
        print("【DUMMY】机械臂回到 HOME 位置：双臂")
        time.sleep(10)

        # self._controller.set_left_gripper(0.0)
        # self._controller.set_right_gripper(0.0)
        print("【DUMMY】夹爪张开：左臂 (0.0)")
        print("【DUMMY】夹爪张开：右臂 (0.0)")
        time.sleep(5)

        # 双臂到目标点
        self.get_logger().info("grasp: 左臂运动到目标点")
        # self._controller.set_cartesian_planing(ARM_ID.LEFT_ARM, left_pose)
        print(f"【DUMMY】左臂笛卡尔规划到目标位置: position={left_pose['position']}, rotation={left_pose['rotation']}")
        time.sleep(5)

        self.get_logger().info("grasp: 右臂运动到目标点")
        # self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        print(f"【DUMMY】右臂笛卡尔规划到目标位置: position={right_pose['position']}, rotation={right_pose['rotation']}")
        time.sleep(5)

        # self._controller.set_left_gripper(1.0)
        # self._controller.set_right_gripper(1.0)
        print("【DUMMY】夹爪闭合：左臂 (1.0)")
        print("【DUMMY】夹爪闭合：右臂 (1.0)")
        time.sleep(5)

        # self._controller.go_zero(ARM_ID.BOTH_ARM)
        print("【DUMMY】机械臂回到 ZERO 位置：双臂")
        time.sleep(5)

    def _do_place(self, request):
        """放置：基于当前位置沿 x 后退 0.15m 放下物品。"""
        # self._controller.set_left_gripper(0.0)
        # self._controller.set_right_gripper(0.0)
        print("【DUMMY】夹爪张开：左臂 (0.0)")
        print("【DUMMY】夹爪张开：右臂 (0.0)")
        time.sleep(5)

        # self._controller.set_cartesian_position_offset_planing(ARM_ID.BOTH_ARM, -0.15, 0.0, 0.0)
        print("【DUMMY】双臂笛卡尔位置偏移规划: x=-0.15, y=0.0, z=0.0")
        time.sleep(5)

        # self._controller.go_zero(ARM_ID.BOTH_ARM)
        print("【DUMMY】机械臂回到 ZERO 位置：双臂")
        time.sleep(5)

    def _do_elevator(self, request):
        """点击电梯：仅右臂动作，含脱离/点击/回归/下沉闭环。"""
        right_pose = self._build_pose(request.right_point, rotation=[0.7071, 0.0, 0.0, 0.7071],
                                      x_offset=-0.2, z_offset=0.02)

        # 夹爪全开
        self.get_logger().info("elevator: set_gripper 0.0")
        # self._controller.set_left_gripper(0.0)
        # self._controller.set_right_gripper(0.0)
        print("【DUMMY】夹爪张开：左臂 (0.0)")
        print("【DUMMY】夹爪张开：右臂 (0.0)")
        time.sleep(2)

        # 机械臂安全脱离小车位置
        self.get_logger().info("elevator: set_cartesian 123")
        # self._controller.set_cartesian_position_offset_planing(ARM_ID.RIGHT_ARM, 0.0, 0.0, 0.1)
        print("【DUMMY】右臂笛卡尔位置偏移规划: x=0.0, y=0.0, z=0.1 (向上脱离)")
        time.sleep(3)

        # 夹爪闭合
        self.get_logger().info("elevator: set_gripper 1.0")
        # self._controller.set_left_gripper(1.0)
        # self._controller.set_right_gripper(1.0)
        print("【DUMMY】夹爪闭合：左臂 (1.0)")
        print("【DUMMY】夹爪闭合：右臂 (1.0)")
        time.sleep(2)

        # 打印脱离位置姿态
        # state = self._controller.get_robot_state()
        # self.get_logger().info(f"elevator: right_tcp_pose xyz={state.right_tcp_pose.position} xyzw={state.right_tcp_pose.rotation}")
        # 模拟状态
        mock_position = [0.5, 0.0, 0.3]
        mock_rotation = [0.7071, 0.0, 0.0, 0.7071]
        self.get_logger().info(f"【DUMMY】elevator: right_tcp_pose xyz={mock_position} xyzw={mock_rotation}")
        time.sleep(2)

        # 保存脱离位置
        # 原始代码使用 POSE() 类
        # target_pose_ww_back = POSE()
        # target_pose_ww_back.position = state.right_tcp_pose.position
        # target_pose_ww_back.rotation = [0.7071, 0.0, 0.0, 0.7071]
        target_pose_ww_back = {
            'position': mock_position,
            'rotation': [0.7071, 0.0, 0.0, 0.7071]
        }
        print(f"【DUMMY】保存脱离位置: {target_pose_ww_back}")

        # 右臂根据视觉坐标点击电梯
        # self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, right_pose)
        print(f"【DUMMY】右臂笛卡尔规划到目标位置（点击电梯）: position={right_pose['position']}, rotation={right_pose['rotation']}")
        time.sleep(3)

        # 右臂回到脱离位置
        self.get_logger().info("elevator: 右臂回到脱离位置")
        # self._controller.set_cartesian_planing(ARM_ID.RIGHT_ARM, target_pose_ww_back)
        print(f"【DUMMY】右臂笛卡尔规划回到脱离位置: position={target_pose_ww_back['position']}, rotation={target_pose_ww_back['rotation']}")
        time.sleep(3)

        # 夹爪全开
        self.get_logger().info("elevator: set_gripper 0.0")
        # self._controller.set_left_gripper(0.0)
        # self._controller.set_right_gripper(0.0)
        print("【DUMMY】夹爪张开：左臂 (0.0)")
        print("【DUMMY】夹爪张开：右臂 (0.0)")
        time.sleep(2)

        # 机械臂下降到小车夹取位置
        # self._controller.set_cartesian_position_offset_planing(ARM_ID.RIGHT_ARM, 0.0, 0.0, -0.1)
        print("【DUMMY】右臂笛卡尔位置偏移规划: x=0.0, y=0.0, z=-0.1 (向下回到小车)")
        time.sleep(1)

        # 夹爪闭合
        self.get_logger().info("elevator: set_gripper 1.0")
        # self._controller.set_left_gripper(1.0)
        # self._controller.set_right_gripper(1.0)
        print("【DUMMY】夹爪闭合：左臂 (1.0)")
        print("【DUMMY】夹爪闭合：右臂 (1.0)")
        time.sleep(2)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    @staticmethod
    def _build_pose(point_stamped, rotation, x_offset=0.0, z_offset=0.0):
        """构建机械臂目标位姿（DUMMY 版本使用字典）"""
        p = point_stamped.point
        # pose = POSE()
        # pose.position = [float(p.x) + x_offset, float(p.y), float(p.z) + z_offset]
        # pose.rotation = list(rotation)
        pose = {
            'position': [float(p.x) + x_offset, float(p.y), float(p.z) + z_offset],
            'rotation': list(rotation)
        }
        return pose

    def destroy_node(self):
        if self._controller is not None:
            try:
                # self._controller.disconnect()
                print("【DUMMY】断开 MosController 连接")
            except Exception as exc:
                self.get_logger().warn(f"断开 MosController 失败: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DummyArmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
