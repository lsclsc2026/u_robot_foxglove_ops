#!/usr/bin/env python3
"""Publish three independent, latest-frame-only JPEG previews for Foxglove."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image


@dataclass(frozen=True)
class CameraConfig:
    name: str
    input_topic: str
    output_topic: str
    width: int
    height: int


class LatestFrameWorker:
    """Encode one camera without ever building a stale frame queue."""

    def __init__(
        self,
        node: Node,
        config: CameraConfig,
        publisher,
        *,
        jpeg_quality: int,
        max_fps: float,
        max_frame_age: float,
    ) -> None:
        self.node = node
        self.config = config
        self.publisher = publisher
        self.jpeg_quality = jpeg_quality
        self.period = 1.0 / max_fps
        self.max_frame_age = max_frame_age
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame_event = threading.Event()
        self.stop_event = threading.Event()
        self.latest: Image | None = None
        self.thread = threading.Thread(
            target=self._run,
            name=f"foxglove_jpeg_{config.name}",
            daemon=True,
        )
        self.thread.start()

    def submit(self, message: Image) -> None:
        # Replacing the reference is intentional: an older unencoded frame is
        # discarded immediately when a newer frame arrives.
        with self.lock:
            self.latest = message
            self.frame_event.set()

    def stop(self) -> None:
        self.stop_event.set()
        self.frame_event.set()
        self.thread.join(timeout=2.0)

    def _take_latest(self) -> Image | None:
        with self.lock:
            message = self.latest
            self.latest = None
            self.frame_event.clear()
            return message

    def _run(self) -> None:
        next_encode = 0.0
        while not self.stop_event.is_set():
            self.frame_event.wait(timeout=0.5)
            if self.stop_event.is_set():
                break

            delay = next_encode - time.monotonic()
            if delay > 0 and self.stop_event.wait(delay):
                break

            # Take the freshest frame only after the FPS wait. Frames received
            # during the wait have therefore already replaced stale ones.
            message = self._take_latest()
            if message is None:
                continue

            # Do no work while the Foxglove panel is closed. This is especially
            # important on the robot because JPEG encoding otherwise competes
            # with localization even though nobody is looking at the stream.
            if self.publisher.get_subscription_count() == 0:
                next_encode = time.monotonic()
                continue

            # A late frame is less useful than the next frame. Drop it before
            # resize/JPEG work so transient CPU or network stalls cannot turn
            # into a growing end-to-end delay.
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            if stamp_ns > 0:
                age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
                if self.max_frame_age > 0.0 and age > self.max_frame_age:
                    next_encode = time.monotonic()
                    continue

            started = time.monotonic()
            try:
                image = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8"
                )
                image = cv2.resize(
                    image,
                    (self.config.width, self.config.height),
                    # INTER_LINEAR is materially cheaper than INTER_AREA on the
                    # Orin and is sufficiently sharp for a live preview.
                    interpolation=cv2.INTER_LINEAR,
                )
                success, encoded = cv2.imencode(
                    ".jpg",
                    image,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not success:
                    self.node.get_logger().warning(
                        f"{self.config.name}: JPEG编码失败"
                    )
                    continue

                output = CompressedImage()
                output.header = message.header
                output.format = "jpeg"
                output.data = encoded.tobytes()
                self.publisher.publish(output)
            except Exception as exc:
                self.node.get_logger().error(
                    f"{self.config.name}: 图像处理失败：{exc}"
                )
            finally:
                # Limit start-to-start rate. Slow encoding never creates a
                # backlog because self.latest contains at most one newer frame.
                next_encode = max(next_encode + self.period, started + self.period)


class FoxgloveImageRelay(Node):
    def __init__(self) -> None:
        super().__init__("foxglove_image_relay")

        self.declare_parameter("jpeg_quality", 58)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("max_frame_age", 0.25)
        self.declare_parameter("head_width", 576)
        self.declare_parameter("head_height", 324)
        self.declare_parameter("arm_width", 432)
        self.declare_parameter("arm_height", 243)

        quality = max(25, min(85, int(self.get_parameter("jpeg_quality").value)))
        max_fps = max(5.0, min(30.0, float(self.get_parameter("max_fps").value)))
        max_frame_age = max(
            0.0, min(2.0, float(self.get_parameter("max_frame_age").value))
        )
        head_width = int(self.get_parameter("head_width").value)
        head_height = int(self.get_parameter("head_height").value)
        arm_width = int(self.get_parameter("arm_width").value)
        arm_height = int(self.get_parameter("arm_height").value)

        # Prevent three encoders from each creating an OpenCV thread pool and
        # causing CPU bursts that compete with localization/navigation.
        cv2.setNumThreads(1)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        configs = (
            CameraConfig(
                "head",
                "/head_camera/color/image_raw",
                "/foxglove/head_camera/image/compressed",
                head_width,
                head_height,
            ),
            CameraConfig(
                "left",
                "/left_arm_camera/color/image_raw",
                "/foxglove/left_arm_camera/image/compressed",
                arm_width,
                arm_height,
            ),
            CameraConfig(
                "right",
                "/right_arm_camera/color/image_raw",
                "/foxglove/right_arm_camera/image/compressed",
                arm_width,
                arm_height,
            ),
        )

        self.workers: list[LatestFrameWorker] = []
        self._input_subscriptions = []
        for config in configs:
            publisher = self.create_publisher(
                CompressedImage, config.output_topic, qos
            )
            worker = LatestFrameWorker(
                self,
                config,
                publisher,
                jpeg_quality=quality,
                max_fps=max_fps,
                max_frame_age=max_frame_age,
            )
            subscription = self.create_subscription(
                Image, config.input_topic, worker.submit, qos
            )
            self.workers.append(worker)
            self._input_subscriptions.append(subscription)
            self.get_logger().info(
                f"{config.name}: {config.input_topic} -> {config.output_topic} "
                f"({config.width}x{config.height})"
            )

        self.get_logger().info(
            "Foxglove最新帧图像节点已启动：JPEG质量=%d，目标帧率=%.1f FPS，"
            "最大帧龄=%.2fs，编码线程=3"
            % (quality, max_fps, max_frame_age)
        )

    def stop(self) -> None:
        for worker in self.workers:
            worker.stop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FoxgloveImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
