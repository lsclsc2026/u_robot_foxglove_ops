#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
# from mos_sdk import MosController

# from mos_vision.srv import MoveChassis
from mos_coordinator.srv import MoveChassis


class DummyChassisNode(Node):
    """底盘服务节点（DUMMY 版本）。

    对外暴露 `~/run_chassis` 服务：
    - 传 target_x_m：按 x - 0.8579 计算前进距离，以 0.05 m/s 前进；
    - 传 return_to_origin=true：以 -0.05 m/s 后退上次前进的距离。
    
    与原始 ww_test_chassis.py 的唯一区别：
    - 所有 self._controller.xxx() 调用已注释并替换为 print 语句
    """

    THRESHOLD_X_M = 0.8579  # 目标 x 阈值
    SPEED_MPS = 0.05        # 固定速度（前进为 +0.05，后退为 -0.05）

    def __init__(self):
        super().__init__("dummy_chassis_node")

        # self._controller = MosController()
        # self._controller.connect()
        self._controller = None
        self.get_logger().info("【DUMMY】已模拟连接 MosController")

        # 记录上次前进距离，供 return_to_origin 使用
        self._last_forward_distance_m = 0.0

        self.run_chassis_service = self.create_service(
            MoveChassis, "~/run_chassis", self._handle_run_chassis
        )
        self.get_logger().info("已提供服务 ~/run_chassis")

    def _handle_run_chassis(self, request, response):
        # 后退到原点
        if request.return_to_origin:
            distance = self._last_forward_distance_m
            if distance <= 0.0:
                response.success = True
                response.message = "无前进记录，无需后退"
                response.actual_distance_m = 0.0
                return response

            speed = -self.SPEED_MPS  # -0.05 后退
            duration = distance / self.SPEED_MPS  # 时间与前进一致

            self.get_logger().info(
                f"后退: distance={distance:.3f} m, speed={speed} m/s, duration={duration:.2f} s"
            )
            # self._controller.set_chassis_velocity(speed, 0, 0)
            print(f"【DUMMY】设置底盘速度: vx={speed:.3f} m/s, vy=0, w=0 (后退)")
            time.sleep(duration)
            # self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
            print("【DUMMY】停止底盘: vx=0, vy=0, w=0")

            self._last_forward_distance_m = 0.0
            response.success = True
            response.message = f"后退完成: {distance:.3f} m"
            response.actual_distance_m = distance
            self.get_logger().info(response.message)
            return response

        # 前进：根据视觉 x 计算距离
        target_x = float(request.target_x_m)
        offset_x = target_x - self.THRESHOLD_X_M

        if offset_x <= 0.0:
            response.success = True
            response.message = (
                f"目标 x={target_x:.4f} 在范围内（阈值 {self.THRESHOLD_X_M}），无需移动"
            )
            response.actual_distance_m = 0.0
            self.get_logger().info(response.message)
            return response

        speed = self.SPEED_MPS  # 0.05 前进
        duration = offset_x / speed

        self.get_logger().info(
            f"前进: target_x={target_x:.4f}, offset={offset_x:.3f} m, "
            f"speed={speed} m/s, duration={duration:.2f} s"
        )
        # self._controller.set_chassis_velocity(speed, 0, 0)
        print(f"【DUMMY】设置底盘速度: vx={speed:.3f} m/s, vy=0, w=0 (前进)")
        time.sleep(duration)
        # self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
        print("【DUMMY】停止底盘: vx=0, vy=0, w=0")

        self._last_forward_distance_m = offset_x
        response.success = True
        response.message = f"前进完成: {offset_x:.3f} m"
        response.actual_distance_m = offset_x
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        if self._controller is not None:
            try:
                # self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
                print("【DUMMY】停止底盘: vx=0, vy=0, w=0")
                # self._controller.disconnect()
                print("【DUMMY】断开 MosController 连接")
            except Exception as exc:
                self.get_logger().warn(f"清理 MosController 失败: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DummyChassisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
