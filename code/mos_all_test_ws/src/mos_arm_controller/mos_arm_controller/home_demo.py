#!/usr/bin/env python3
import argparse


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

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Call the fused hardware controller service to send arms home"
    )
    parser.add_argument(
        "--service-wait-sec",
        type=float,
        default=10.0,
        help="seconds to wait for /mos_arm_controller/run_task to appear",
    )
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from mos_arm_controller.srv import RunTask

    class ArmHomeDemoClient(Node):
        def __init__(self):
            super().__init__("mos_arm_home_demo")
            self._client = self.create_client(RunTask, "/mos_arm_controller/run_task")

        def run(self, wait_timeout_sec):
            self.get_logger().info("waiting for /mos_arm_controller/run_task service")
            if not self._client.wait_for_service(timeout_sec=wait_timeout_sec):
                raise RuntimeError("/mos_arm_controller/run_task service not available")

            request = fill_home_request(RunTask.Request())
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            response = future.result()
            if response is None:
                raise RuntimeError("home request returned no response")
            if not response.success:
                raise RuntimeError(f"home task failed: {response.message}")

            self.get_logger().info(f"home task finished: {response.message}")

    rclpy.init()
    node = ArmHomeDemoClient()
    try:
        node.run(wait_timeout_sec=args.service_wait_sec)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
