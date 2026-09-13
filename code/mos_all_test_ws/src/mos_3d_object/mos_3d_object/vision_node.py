"""ROS2 node for online MOS RGB-D YOLO 3D object coordinate detection."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from .coordinate_transformer import CoordinateTransformer
from .image_messages import bgr_to_image_message
from .rgbd_localizer import RGBDObjectLocalizer

try:
    from mos_3d_object.msg import DetectedObject3D, DetectedObject3DArray
except ImportError:
    DetectedObject3D = None
    DetectedObject3DArray = None


CAMERA_ENUM_NAMES = {
    "head": "HEAD_CAMERA",
    "left_arm": "LEFT_ARM_CAMERA",
    "right_arm": "RIGHT_ARM_CAMERA",
}


def header_value(data, name, default=None):
    header = getattr(data, "header", None)
    return getattr(header, name, default) if header is not None else default


def sdk_intrinsic_to_dict(intrinsic):
    if intrinsic is None:
        raise RuntimeError("MOS get_color_intrinsics() returned None")
    coeffs = list(getattr(intrinsic, "coeffs", []))
    coeffs.extend([0.0] * (5 - len(coeffs)))
    return {
        "Fx": float(intrinsic.fx),
        "Fy": float(intrinsic.fy),
        "Cx": float(intrinsic.cx),
        "Cy": float(intrinsic.cy),
        "k1": float(coeffs[0]),
        "k2": float(coeffs[1]),
        "p1": float(coeffs[2]),
        "p2": float(coeffs[3]),
        "k3": float(coeffs[4]),
    }


def depth_to_meters(depth_data, depth_raw, fallback_scale=None):
    scale = getattr(depth_data, "depth_scale", None)
    if scale is None:
        scale = fallback_scale
    if scale is None:
        raise RuntimeError("DepthData has no depth_scale; set depth_scale parameter")
    scale = float(scale)
    if not np.isfinite(scale) or scale <= 0.0:
        raise RuntimeError(f"invalid depth scale: {scale}")
    if depth_raw.ndim == 3:
        depth_raw = depth_raw[:, :, 0]
    meters_per_count = 1000.0 / scale
    return depth_raw.astype(np.float32) * meters_per_count, scale, meters_per_count


class Mos3DObjectNode(Node):
    def __init__(self):
        super().__init__("mos_3d_object")
        self._declare_parameters()
        self._load_parameters()

        try:
            from mos_sdk import CameraType, MosSensorAdapter
        except ImportError as exc:
            raise RuntimeError(
                "mos_sdk is unavailable. Run this node inside the Lumos/MOS SDK environment."
            ) from exc

        if self.camera not in CAMERA_ENUM_NAMES:
            raise ValueError(f"unsupported camera: {self.camera}")

        self.camera_type = getattr(CameraType, CAMERA_ENUM_NAMES[self.camera])
        self.head_camera_type = getattr(CameraType, CAMERA_ENUM_NAMES["head"])
        self.left_arm_camera_type = getattr(
            CameraType, CAMERA_ENUM_NAMES["left_arm"]
        )
        self.right_arm_camera_type = getattr(
            CameraType, CAMERA_ENUM_NAMES["right_arm"]
        )
        self.sensor = MosSensorAdapter(self.config)
        # The SDK publisher is the efficient, native path for RGB-D, LiDAR,
        # and IMU ROS topics.  It must be enabled before start() creates the
        # underlying publishers and acquisition threads.
        self.sensor.enable_ros_publisher(self.enable_sensor_ros_publisher)
        if not self.sensor.start():
            raise RuntimeError("MosSensorAdapter.start() failed")

        color_intrinsic = self.sensor.get_color_intrinsics(self.camera_type)
        intrinsic_data = sdk_intrinsic_to_dict(color_intrinsic)
        self.localizer = RGBDObjectLocalizer(
            rgb_intrinsic_data=intrinsic_data,
            depth_intrinsic_data=intrinsic_data,
            depth_is_rgb_aligned=True,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
            model_size=self.model_size,
            weights=self.weights or None,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            target_classes=self.target_classes or None,
        )

        self.transformer = CoordinateTransformer(
            self,
            use_tf=self.use_tf,
            target_frame=self.target_frame,
            camera_frame=self.camera_frame,
            camera_name=self.camera,
            static_extrinsic_file=self.static_extrinsic_file,
            invert_static_extrinsic=self.invert_static_extrinsic,
            tf_timeout_sec=self.tf_timeout_sec,
            use_latest_tf=self.use_latest_tf,
        )

        self.pub_detections_3d = None
        if self.publish_detections_3d and DetectedObject3DArray is not None:
            self.pub_detections_3d = self.create_publisher(
                DetectedObject3DArray, self.detections_3d_topic, 10
            )
        elif self.publish_detections_3d:
            self.get_logger().warn(
                "DetectedObject3DArray message is unavailable; "
                "/vision/detections_3d will not be published until the package is rebuilt"
            )

        self.pub_head_rgb = None
        self.pub_left_arm_rgb = None
        self.pub_right_arm_rgb = None
        self._rgb_sources = ()
        self.rgb_publish_timer = None
        if self.enable_python_rgb_publisher:
            self.pub_head_rgb = self.create_publisher(
                Image, self.head_rgb_topic, qos_profile_sensor_data
            )
            self.pub_left_arm_rgb = self.create_publisher(
                Image, self.left_rgb_topic, qos_profile_sensor_data
            )
            self.pub_right_arm_rgb = self.create_publisher(
                Image, self.right_rgb_topic, qos_profile_sensor_data
            )
            self._rgb_sources = (
                (
                    "head",
                    self.head_camera_type,
                    self.pub_head_rgb,
                    self.head_rgb_frame_id,
                ),
                (
                    "left_arm",
                    self.left_arm_camera_type,
                    self.pub_left_arm_rgb,
                    self.left_rgb_frame_id,
                ),
                (
                    "right_arm",
                    self.right_arm_camera_type,
                    self.pub_right_arm_rgb,
                    self.right_rgb_frame_id,
                ),
            )
            self.rgb_publish_timer = self.create_timer(
                self.rgb_publish_seconds, self._publish_rgb_frames
            )
        self.processed = 0
        self.srv_set_enabled = self.create_service(
            SetBool, "~/set_enabled", self._handle_set_enabled
        )
        self.timer = self.create_timer(self.poll_seconds, self._tick)
        self.get_logger().info(
            f"started MOS vision node camera={self.camera} use_tf={self.use_tf} "
            f"target_frame={self.target_frame} enabled={self.enabled}; "
            f"sensor_ros_publisher={self.enable_sensor_ros_publisher} "
            f"python_rgb_publisher={self.enable_python_rgb_publisher}; RGB topics: "
            f"head={self.head_rgb_topic}, left={self.left_rgb_topic}, "
            f"right={self.right_rgb_topic}"
        )

    def _declare_parameters(self):
        defaults = {
            "config": "/opt/lumos/config/mos_sensor/example.yaml",
            "camera": "head",
            "weights": "",
            "model_size": "nano",
            "enabled": False,
            "target_classes": "",
            "conf": 0.25,
            "iou": 0.45,
            "device": "cpu",
            "min_depth_m": 0.15,
            "max_depth_m": 12.0,
            "depth_scale": 0.0,
            "poll_seconds": 0.05,
            "track": True,
            "max_rgb_depth_delta_ms": 0.0,
            "use_tf": True,
            "target_frame": "base_link",
            "camera_frame": "",
            "tf_timeout_sec": 0.1,
            "use_latest_tf": True,
            "static_extrinsic_file": "",
            "invert_static_extrinsic": False,
            "detections_3d_topic": "/vision/detections_3d",
            "publish_detections_3d": True,
            "enable_sensor_ros_publisher": True,
            "enable_python_rgb_publisher": False,
            "rgb_publish_seconds": 0.05,
            "head_rgb_topic": "/head_camera/color/image_raw",
            "left_rgb_topic": "/left_arm_camera/color/image_raw",
            "right_rgb_topic": "/right_arm_camera/color/image_raw",
            "head_rgb_frame_id": "head_camera_optical_frame",
            "left_rgb_frame_id": "left_arm_camera_optical_frame",
            "right_rgb_frame_id": "right_arm_camera_optical_frame",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.config = self.get_parameter("config").value
        self.camera = self.get_parameter("camera").value
        self.weights = self.get_parameter("weights").value
        self.model_size = self.get_parameter("model_size").value
        self.enabled = bool(self.get_parameter("enabled").value)
        self.target_classes = self.get_parameter("target_classes").value
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.device = self.get_parameter("device").value
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_scale = depth_scale if depth_scale > 0.0 else None
        self.poll_seconds = float(self.get_parameter("poll_seconds").value)
        self.track = bool(self.get_parameter("track").value)
        max_delta = float(self.get_parameter("max_rgb_depth_delta_ms").value)
        self.max_rgb_depth_delta_ms = max_delta if max_delta > 0.0 else None
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.static_extrinsic_file = self.get_parameter("static_extrinsic_file").value
        self.invert_static_extrinsic = bool(
            self.get_parameter("invert_static_extrinsic").value
        )
        self.detections_3d_topic = self.get_parameter("detections_3d_topic").value
        self.publish_detections_3d = bool(
            self.get_parameter("publish_detections_3d").value
        )
        self.enable_sensor_ros_publisher = bool(
            self.get_parameter("enable_sensor_ros_publisher").value
        )
        self.enable_python_rgb_publisher = bool(
            self.get_parameter("enable_python_rgb_publisher").value
        )
        self.rgb_publish_seconds = float(
            self.get_parameter("rgb_publish_seconds").value
        )
        self.head_rgb_topic = str(self.get_parameter("head_rgb_topic").value)
        self.left_rgb_topic = str(self.get_parameter("left_rgb_topic").value)
        self.right_rgb_topic = str(self.get_parameter("right_rgb_topic").value)
        self.head_rgb_frame_id = str(
            self.get_parameter("head_rgb_frame_id").value
        )
        self.left_rgb_frame_id = str(
            self.get_parameter("left_rgb_frame_id").value
        )
        self.right_rgb_frame_id = str(
            self.get_parameter("right_rgb_frame_id").value)
        if self.enable_python_rgb_publisher and self.rgb_publish_seconds <= 0.0:
            raise ValueError("rgb_publish_seconds must be positive")

    def _publish_rgb_frames(self):
        stamp = self.get_clock().now().to_msg()
        for camera_name, camera_type, publisher, frame_id in self._rgb_sources:
            try:
                color = self.sensor.get_image_data(camera_type)
                if color is None:
                    continue
                frame = color.convert_to_cv2_mat()
                if frame is None or frame.size == 0:
                    continue
                publisher.publish(
                    bgr_to_image_message(
                        frame,
                        stamp=stamp,
                        frame_id=frame_id,
                        image_type=Image,
                    )
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"skip {camera_name} RGB publish: {exc}",
                    throttle_duration_sec=5.0,
                )

    def _tick(self):
        if not self.enabled:
            return

        frame_index = self.processed + 1
        tick_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=tick event=start"
        )

        get_rgbd_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=get_rgbd event=start"
        )
        pair = self.sensor.get_rgbd_data(self.camera_type)
        get_rgbd_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=get_rgbd event=end "
            f"duration_ms={(get_rgbd_t1 - get_rgbd_t0) * 1000.0:.1f}"
        )
        if not pair:
            self.get_logger().info(
                f"TIMING mos_3d_object_drop frame={frame_index} reason=no_rgbd_pair"
            )
            return
        rgb, depth = pair
        if rgb is None or depth is None:
            self.get_logger().info(
                f"TIMING mos_3d_object_drop frame={frame_index} reason=missing_rgb_or_depth"
            )
            return

        convert_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=convert_rgbd event=start"
        )
        image = rgb.convert_to_cv2_mat()
        depth_raw = depth.convert_to_cv2_mat()
        convert_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=convert_rgbd event=end "
            f"duration_ms={(convert_t1 - convert_t0) * 1000.0:.1f}"
        )
        if image is None or depth_raw is None or image.size == 0 or depth_raw.size == 0:
            self.get_logger().info(
                f"TIMING mos_3d_object_drop frame={frame_index} reason=empty_rgb_or_depth"
            )
            return
        self.get_logger().info(
            f"TIMING mos_3d_object_image frame={frame_index} "
            f"rgb_shape={image.shape} rgb_dtype={image.dtype} "
            f"depth_shape={depth_raw.shape} depth_dtype={depth_raw.dtype}"
        )
        if image.shape[:2] != depth_raw.shape[:2]:
            self.get_logger().warn(
                f"skip unaligned frame: RGB {image.shape[:2]}, depth {depth_raw.shape[:2]}"
            )
            self.get_logger().info(
                f"TIMING mos_3d_object_drop frame={frame_index} reason=unaligned_rgb_depth"
            )
            return

        sync_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=rgb_depth_sync event=start"
        )
        rgb_timestamp_ns = header_value(rgb, "timestamp_ns")
        depth_timestamp_ns = header_value(depth, "timestamp_ns")
        delta_ms = None
        if rgb_timestamp_ns is not None and depth_timestamp_ns is not None:
            delta_ms = abs(int(rgb_timestamp_ns) - int(depth_timestamp_ns)) / 1e6
            if (
                self.max_rgb_depth_delta_ms is not None
                and delta_ms > self.max_rgb_depth_delta_ms
            ):
                self.get_logger().warn(
                    f"skip RGB-D pair with {delta_ms:.3f} ms timestamp delta"
                )
                self.get_logger().info(
                    f"TIMING mos_3d_object_drop frame={frame_index} "
                    f"reason=rgb_depth_delta_too_large delta_ms={delta_ms:.3f}"
                )
                return
        sync_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=rgb_depth_sync event=end "
            f"duration_ms={(sync_t1 - sync_t0) * 1000.0:.1f} "
            f"delta_ms={delta_ms if delta_ms is not None else -1:.3f}"
        )

        timestamp_ns = rgb_timestamp_ns if rgb_timestamp_ns is not None else depth_timestamp_ns
        camera_frame = (
            self.camera_frame
            or header_value(rgb, "frame_id")
            or CAMERA_ENUM_NAMES[self.camera]
        )
        transform_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=resolve_transform event=start"
        )
        try:
            T_target_camera, target_frame, camera_frame, transform_source = (
                self.transformer.resolve(camera_frame, timestamp_ns)
            )
        except Exception as exc:
            if self.use_tf:
                self.get_logger().warn(f"TF lookup failed: {exc}")
            T_target_camera = None
            target_frame = self.target_frame
            transform_source = None
        transform_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=resolve_transform event=end "
            f"duration_ms={(transform_t1 - transform_t0) * 1000.0:.1f}"
        )

        depth_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=depth_to_meters event=start"
        )
        depth_m, used_depth_scale, depth_unit_m = depth_to_meters(
            depth, depth_raw, fallback_scale=self.depth_scale
        )
        depth_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=depth_to_meters event=end "
            f"duration_ms={(depth_t1 - depth_t0) * 1000.0:.1f}"
        )

        localize_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=localize event=start"
        )
        records = self.localizer.localize(
            image,
            depth_m,
            timestamp=timestamp_ns,
            track=self.track,
            T_target_camera=T_target_camera,
            target_frame=target_frame,
            camera_frame=camera_frame,
            transform_source=transform_source,
        )
        localize_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=localize event=end "
            f"duration_ms={(localize_t1 - localize_t0) * 1000.0:.1f} "
            f"detections={len(records)}"
        )

        for record in records:
            record["timestamp_ns"] = int(timestamp_ns) if timestamp_ns is not None else None
            record["rgb_timestamp_ns"] = (
                int(rgb_timestamp_ns) if rgb_timestamp_ns is not None else None
            )
            record["depth_timestamp_ns"] = (
                int(depth_timestamp_ns) if depth_timestamp_ns is not None else None
            )
            record["rgb_depth_delta_ms"] = delta_ms
            record["depth_frame"] = header_value(depth, "frame_id")
            record["depth_scale"] = used_depth_scale
            record["depth_unit_m"] = depth_unit_m
            record["camera_type"] = CAMERA_ENUM_NAMES[self.camera]

        publish_t0 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=publish_detections_3d event=start"
        )
        self._publish_detections_3d(records, camera_frame, target_frame)
        publish_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_stage frame={frame_index} stage=publish_detections_3d event=end "
            f"duration_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
            f"detections={len(records)}"
        )

        self.processed += 1
        tick_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING mos_3d_object_summary frame={frame_index} "
            f"get_rgbd_ms={(get_rgbd_t1 - get_rgbd_t0) * 1000.0:.1f} "
            f"convert_ms={(convert_t1 - convert_t0) * 1000.0:.1f} "
            f"sync_ms={(sync_t1 - sync_t0) * 1000.0:.1f} "
            f"transform_ms={(transform_t1 - transform_t0) * 1000.0:.1f} "
            f"depth_ms={(depth_t1 - depth_t0) * 1000.0:.1f} "
            f"localize_ms={(localize_t1 - localize_t0) * 1000.0:.1f} "
            f"publish_detections_3d_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
            f"total_ms={(tick_t1 - tick_t0) * 1000.0:.1f} "
            f"detections={len(records)}"
        )

    def _handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        state = "enabled" if self.enabled else "disabled"
        response.success = True
        response.message = f"mos_3d_object {state}"
        self.get_logger().info(response.message)
        return response

    def _publish_detections_3d(self, records, camera_frame, target_frame):
        if self.pub_detections_3d is None:
            return

        stamp = self.get_clock().now().to_msg()
        msg = DetectedObject3DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = str(target_frame or self.target_frame or "")
        msg.camera_name = str(self.camera or "")
        msg.camera_frame = str(camera_frame or self.camera_frame or "")
        msg.target_frame = str(target_frame or self.target_frame or "")
        msg.detections = [
            self._record_to_detection_msg(record, stamp, msg.camera_frame, msg.target_frame)
            for record in records
        ]
        self.pub_detections_3d.publish(msg)

    def _record_to_detection_msg(self, record, stamp, camera_frame, target_frame):
        # 将内部 dict 转成正式 ROS2 检测消息。
        msg = DetectedObject3D()
        msg.header.stamp = stamp
        msg.header.frame_id = str(record.get("target_frame") or target_frame or "")
        msg.class_id = int(record.get("class_id", -1))
        msg.class_name = str(record.get("class_name") or "")
        msg.score = float(record.get("score", 0.0))
        object_id = record.get("object_id")
        msg.object_id = int(object_id) if object_id is not None else -1

        bbox = list(record.get("bbox_xyxy") or [0.0, 0.0, 0.0, 0.0])
        bbox.extend([0.0] * (4 - len(bbox)))
        msg.bbox_xyxy = [float(value) for value in bbox[:4]]

        msg.position_camera = self._make_point(
            record.get("position_camera_m") or [float("nan")] * 3,
            record.get("camera_frame") or camera_frame,
            stamp,
        )

        target_position = record.get("position_target_m")
        msg.has_target_position = target_position is not None
        msg.position_target = self._make_point(
            target_position or [float("nan")] * 3,
            record.get("target_frame") or target_frame,
            stamp,
        )
        msg.target_frame = str(record.get("target_frame") or target_frame or "")
        msg.depth_m = float(record.get("depth_m", float("nan")))
        return msg

    @staticmethod
    def _make_point(position, frame_id, stamp):
        msg = PointStamped()
        msg.header.frame_id = str(frame_id or "")
        msg.header.stamp = stamp
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])
        return msg

    def destroy_node(self):
        if hasattr(self, "sensor") and self.sensor is not None:
            self.sensor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mos3DObjectNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
