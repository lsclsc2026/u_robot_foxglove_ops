#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from mos_sdk import MosController

from mos_chassis_controller.srv import MoveChassis


class ChassisControllerNode(Node):
    """Chassis service node used by mos_grasp_pipeline."""

    THRESHOLD_X_M = 0.8579
    SPEED_MPS = 0.05

    def __init__(self):
        super().__init__("mos_chassis_controller")

        self._controller = MosController()
        self._controller.connect()
        self.get_logger().info("connected MosController")

        self._last_forward_distance_m = 0.0

        self.run_chassis_service = self.create_service(
            MoveChassis, "~/run_chassis", self._handle_run_chassis
        )
        self.get_logger().info("ready ~/run_chassis")

    def _handle_run_chassis(self, request, response):
        if request.return_to_origin:
            return self._return_to_origin(response)

        target_x = float(request.target_x_m)
        offset_x = target_x - self.THRESHOLD_X_M

        if offset_x <= 0.0:
            response.success = True
            response.message = (
                f"target x={target_x:.4f} within threshold {self.THRESHOLD_X_M:.4f}; "
                "no chassis movement"
            )
            response.actual_distance_m = 0.0
            self.get_logger().info(response.message)
            return response

        duration = offset_x / self.SPEED_MPS
        self.get_logger().info(
            f"move forward: target_x={target_x:.4f}, "
            f"distance={offset_x:.3f} m, speed={self.SPEED_MPS:.3f} m/s, "
            f"duration={duration:.2f} s"
        )
        self._controller.set_chassis_velocity(self.SPEED_MPS, 0.0, 0.0)
        time.sleep(duration)
        self._controller.set_chassis_velocity(0.0, 0.0, 0.0)

        self._last_forward_distance_m = offset_x
        response.success = True
        response.message = f"forward complete: {offset_x:.3f} m"
        response.actual_distance_m = offset_x
        self.get_logger().info(response.message)
        return response

    def _return_to_origin(self, response):
        distance = self._last_forward_distance_m
        if distance <= 0.0:
            response.success = True
            response.message = "no recorded forward movement"
            response.actual_distance_m = 0.0
            self.get_logger().info(response.message)
            return response

        duration = distance / self.SPEED_MPS
        speed = -self.SPEED_MPS
        self.get_logger().info(
            f"move backward: distance={distance:.3f} m, "
            f"speed={speed:.3f} m/s, duration={duration:.2f} s"
        )
        self._controller.set_chassis_velocity(speed, 0.0, 0.0)
        time.sleep(duration)
        self._controller.set_chassis_velocity(0.0, 0.0, 0.0)

        self._last_forward_distance_m = 0.0
        response.success = True
        response.message = f"backward complete: {distance:.3f} m"
        response.actual_distance_m = distance
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        if self._controller is not None:
            try:
                self._controller.set_chassis_velocity(0.0, 0.0, 0.0)
                self._controller.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"failed to cleanup MosController: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
