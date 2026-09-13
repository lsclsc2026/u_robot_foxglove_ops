#!/usr/bin/env python3
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from mos_arm_controller.srv import RunTask
from mos_chassis_controller.srv import MoveChassis
from mos_grasp_selector.msg import GraspTaskArray


@dataclass
class GraspPair:
    left: object
    right: object
    frame: str


class MosGraspPipeline(Node):
    def __init__(self):
        super().__init__("mos_grasp_pipeline")
        self._declare_parameters()
        self._load_parameters()

        self._task_condition = threading.Condition()
        self._latest_task = None
        self._running = False
        self._worker = None

        self.sub_tasks = self.create_subscription(
            GraspTaskArray,
            self.grasp_tasks_topic,
            self._on_grasp_tasks,
            10,
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.vision_set_enabled_client = self.create_client(
            SetBool, self.vision_set_enabled_service
        )
        self.selector_set_enabled_client = self.create_client(
            SetBool, self.grasp_selector_set_enabled_service
        )
        self.selector_reset_client = self.create_client(
            Trigger, self.grasp_selector_reset_service
        )
        self.arm_run_task_client = self.create_client(
            RunTask, self.arm_run_task_service
        )
        self.chassis_run_chassis_client = self.create_client(
            MoveChassis, self.chassis_run_chassis_service
        )

        self.start_service = self.create_service(Trigger, "~/start", self._handle_start)

        self._auto_start_timer = None
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.5, self._handle_auto_start)

        self.get_logger().info(
            "started mos_grasp_pipeline; call /mos_grasp_pipeline/start after navigation stops"
        )
        self._publish_status("idle")

    def _declare_parameters(self):
        defaults = {
            "auto_start": False,
            "grasp_tasks_topic": "/vision/grasp_tasks",
            "vision_set_enabled_service": "/mos_3d_object/set_enabled",
            "grasp_selector_set_enabled_service": "/mos_grasp_selector/set_enabled",
            "grasp_selector_reset_service": "/mos_grasp_selector/reset",
            "arm_run_task_service": "/mos_arm_controller/run_task",
            "chassis_run_chassis_service": "/mos_chassis_controller/run_chassis",
            "target_frame": "body_link",
            "first_grasp_timeout_sec": 30.0,
            "second_grasp_timeout_sec": 30.0,
            "service_timeout_sec": 60.0,
            "post_chassis_settle_sec": 0.8,
            "return_after_grasp": True,
            "task_name": "grasp",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.grasp_tasks_topic = self.get_parameter("grasp_tasks_topic").value
        self.vision_set_enabled_service = self.get_parameter(
            "vision_set_enabled_service"
        ).value
        self.grasp_selector_set_enabled_service = self.get_parameter(
            "grasp_selector_set_enabled_service"
        ).value
        self.grasp_selector_reset_service = self.get_parameter(
            "grasp_selector_reset_service"
        ).value
        self.arm_run_task_service = self.get_parameter("arm_run_task_service").value
        self.chassis_run_chassis_service = self.get_parameter(
            "chassis_run_chassis_service"
        ).value
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.first_grasp_timeout_sec = float(
            self.get_parameter("first_grasp_timeout_sec").value
        )
        self.second_grasp_timeout_sec = float(
            self.get_parameter("second_grasp_timeout_sec").value
        )
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.post_chassis_settle_sec = float(
            self.get_parameter("post_chassis_settle_sec").value
        )
        self.return_after_grasp = bool(self.get_parameter("return_after_grasp").value)
        self.task_name = str(self.get_parameter("task_name").value)

    def _handle_auto_start(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        self._start_pipeline("auto_start")

    def _handle_start(self, request, response):
        del request
        started = self._start_pipeline("start_service")
        response.success = started
        response.message = (
            "mos_grasp_pipeline started"
            if started
            else "mos_grasp_pipeline is already running"
        )
        return response

    def _start_pipeline(self, source):
        if self._running:
            return False
        self._running = True
        self._publish_status("running")
        self._worker = threading.Thread(
            target=self._run_pipeline,
            args=(source,),
            daemon=True,
        )
        self._worker.start()
        return True

    def _publish_status(self, status):
        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)
        self.get_logger().info(f"pipeline status: {msg.data}")

    def _on_grasp_tasks(self, msg):
        grasp_pair = self._extract_ready_grasp_pair(msg)
        if grasp_pair is None:
            return
        with self._task_condition:
            self._latest_task = grasp_pair
            self._task_condition.notify_all()

    def _extract_ready_grasp_pair(self, msg):
        if not msg.tasks:
            return None

        task = msg.tasks[0]
        if not bool(task.ready):
            return None
        if task.grasp_mode != "dual_arm_sync":
            return None
        if self.target_frame and task.target_frame != self.target_frame:
            self.get_logger().warn(
                f"ignore grasp task with frame={task.target_frame}, expected={self.target_frame}"
            )
            return None

        left = None
        right = None
        for point in task.points:
            if point.role == "left_handle":
                left = point.point.point
            elif point.role == "right_handle":
                right = point.point.point

        if left is None or right is None:
            return None
        if not self._point_is_finite(left) or not self._point_is_finite(right):
            return None

        return GraspPair(left=left, right=right, frame=task.target_frame)

    @staticmethod
    def _point_is_finite(point):
        return (
            math.isfinite(float(point.x))
            and math.isfinite(float(point.y))
            and math.isfinite(float(point.z))
        )

    def _clear_latest_task(self):
        with self._task_condition:
            self._latest_task = None

    def _wait_for_grasp_pair(self, timeout_sec, label):
        deadline = time.monotonic() + float(timeout_sec)
        with self._task_condition:
            while self._latest_task is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"timeout waiting for {label} stable grasp task")
                self._task_condition.wait(timeout=min(0.2, remaining))
            grasp_pair = self._latest_task
            self._latest_task = None
            return grasp_pair

    def _run_pipeline(self, source):
        self.get_logger().info(f"pipeline started by {source}")
        try:
            first_pair = self._capture_stable_grasp_pair(
                self.first_grasp_timeout_sec, "first"
            )
            self._set_selector_enabled(False)
            self._set_vision_enabled(False)

            self._move_chassis_from_first_pair(first_pair)
            time.sleep(self.post_chassis_settle_sec)

            second_pair = self._capture_stable_grasp_pair(
                self.second_grasp_timeout_sec, "second"
            )
            self._call_arm_run_task(self.task_name, second_pair)

            self._set_selector_enabled(False)
            self._set_vision_enabled(False)
            if self.return_after_grasp:
                self._move_chassis_back_to_origin()
            self.get_logger().info("pipeline finished successfully")
            self._publish_status("succeeded")
        except Exception as exc:
            self.get_logger().error(f"pipeline failed: {exc}")
            self._publish_status("failed")
            self._stop_chassis_safely()
            self._disable_vision_safely()
        finally:
            self._running = False

    def _capture_stable_grasp_pair(self, timeout_sec, label):
        self.get_logger().info(f"capture {label} stable grasp pair")
        self._clear_latest_task()
        self._reset_selector()
        self._set_vision_enabled(True)
        self._set_selector_enabled(True)
        grasp_pair = self._wait_for_grasp_pair(timeout_sec, label)
        self.get_logger().info(
            f"{label} grasp pair: "
            f"left=({grasp_pair.left.x:.4f},{grasp_pair.left.y:.4f},{grasp_pair.left.z:.4f}) "
            f"right=({grasp_pair.right.x:.4f},{grasp_pair.right.y:.4f},{grasp_pair.right.z:.4f})"
        )
        return grasp_pair

    def _move_chassis_from_first_pair(self, grasp_pair):
        handle_x = (float(grasp_pair.left.x) + float(grasp_pair.right.x)) * 0.5
        self.get_logger().info(
            f"request chassis forward with handle_x={handle_x:.4f}"
        )
        request = MoveChassis.Request()
        request.target_x_m = float(handle_x)
        request.return_to_origin = False
        response = self._call_service(
            self.chassis_run_chassis_client, request, "mos_chassis_controller/run_chassis"
        )
        if not response.success:
            raise RuntimeError(f"chassis forward failed: {response.message}")
        self.get_logger().info(
            f"chassis forward response: {response.message}, "
            f"actual_distance_m={response.actual_distance_m:.3f}"
        )

    def _move_chassis_back_to_origin(self):
        self.get_logger().info("request chassis return to origin")
        request = MoveChassis.Request()
        request.return_to_origin = True
        response = self._call_service(
            self.chassis_run_chassis_client, request, "mos_chassis_controller/run_chassis"
        )
        if not response.success:
            raise RuntimeError(f"chassis return failed: {response.message}")
        self.get_logger().info(
            f"chassis return response: {response.message}, "
            f"actual_distance_m={response.actual_distance_m:.3f}"
        )

    def _call_arm_run_task(self, task_name, grasp_pair):
        request = RunTask.Request()
        request.task_name = str(task_name)
        request.target_frame = str(grasp_pair.frame)
        request.left_point = self._to_point_stamped(grasp_pair.left, grasp_pair.frame)
        request.right_point = self._to_point_stamped(grasp_pair.right, grasp_pair.frame)
        self.get_logger().info(
            f"request arm task={task_name} "
            f"left=({request.left_point.point.x:.4f},"
            f"{request.left_point.point.y:.4f},"
            f"{request.left_point.point.z:.4f}) "
            f"right=({request.right_point.point.x:.4f},"
            f"{request.right_point.point.y:.4f},"
            f"{request.right_point.point.z:.4f})"
        )
        response = self._call_service(
            self.arm_run_task_client, request, "mos_arm_controller/run_task"
        )
        if not response.success:
            raise RuntimeError(f"arm run_task failed: {response.message}")
        self.get_logger().info(f"arm task done: {response.message}")

    @staticmethod
    def _to_point_stamped(point, frame_id):
        msg = PointStamped()
        msg.header.frame_id = str(frame_id or "")
        msg.point.x = float(point.x)
        msg.point.y = float(point.y)
        msg.point.z = float(point.z)
        return msg

    def _set_vision_enabled(self, enabled):
        request = SetBool.Request()
        request.data = bool(enabled)
        self._call_service(self.vision_set_enabled_client, request, "mos_3d_object/set_enabled")

    def _set_selector_enabled(self, enabled):
        request = SetBool.Request()
        request.data = bool(enabled)
        self._call_service(
            self.selector_set_enabled_client,
            request,
            "mos_grasp_selector/set_enabled",
        )

    def _reset_selector(self):
        request = Trigger.Request()
        self._call_service(self.selector_reset_client, request, "mos_grasp_selector/reset")

    def _call_service(self, client, request, label):
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise TimeoutError(f"service unavailable: {label}")
        future = client.call_async(request)
        deadline = time.monotonic() + self.service_timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"service timeout: {label}")
            time.sleep(0.05)
        response = future.result()
        if response is None:
            raise RuntimeError(f"service failed without response: {label}")
        if hasattr(response, "success") and not bool(response.success):
            raise RuntimeError(f"service returned failure: {label}: {response.message}")
        return response

    def _stop_chassis_safely(self):
        self.get_logger().warn("skip chassis stop: no stop command in service interface")

    def _disable_vision_safely(self):
        try:
            self._set_selector_enabled(False)
        except Exception as exc:
            self.get_logger().warn(f"failed to disable grasp selector: {exc}")
        try:
            self._set_vision_enabled(False)
        except Exception as exc:
            self.get_logger().warn(f"failed to disable vision: {exc}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MosGraspPipeline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
