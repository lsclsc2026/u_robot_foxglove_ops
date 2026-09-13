#!/usr/bin/env python3
"""ROS2 dummy node for testing MOS 3D object detection without hardware."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_srvs.srv import SetBool

# from .coordinate_transformer import CoordinateTransformer
# from .rgbd_localizer import RGBDObjectLocalizer

try:
    from mos_3d_object.msg import DetectedObject3D, DetectedObject3DArray
except ImportError:
    print("【DUMMY】警告: mos_3d_object.msg 无法导入，请确保已编译该包")
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
    # if intrinsic is None:
    #     raise RuntimeError("MOS get_color_intrinsics() returned None")
    # coeffs = list(getattr(intrinsic, "coeffs", []))
    # coeffs.extend([0.0] * (5 - len(coeffs)))
    # return {
    #     "Fx": float(intrinsic.fx),
    #     "Fy": float(intrinsic.fy),
    #     "Cx": float(intrinsic.cx),
    #     "Cy": float(intrinsic.cy),
    #     "k1": float(coeffs[0]),
    #     "k2": float(coeffs[1]),
    #     "p1": float(coeffs[2]),
    #     "p2": float(coeffs[3]),
    #     "k3": float(coeffs[4]),
    # }
    print("【DUMMY】跳过 sdk_intrinsic_to_dict，返回模拟内参")
    return {
        "Fx": 600.0,
        "Fy": 600.0,
        "Cx": 320.0,
        "Cy": 240.0,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "k3": 0.0,
    }


def depth_to_meters(depth_data, depth_raw, fallback_scale=None):
    # scale = getattr(depth_data, "depth_scale", None)
    # if scale is None:
    #     scale = fallback_scale
    # if scale is None:
    #     raise RuntimeError("DepthData has no depth_scale; set depth_scale parameter")
    # scale = float(scale)
    # if not np.isfinite(scale) or scale <= 0.0:
    #     raise RuntimeError(f"invalid depth scale: {scale}")
    # if depth_raw.ndim == 3:
    #     depth_raw = depth_raw[:, :, 0]
    # meters_per_count = 1000.0 / scale
    # return depth_raw.astype(np.float32) * meters_per_count, scale, meters_per_count
    # print("【DUMMY】跳过 depth_to_meters，返回模拟深度数据")
    scale = fallback_scale if fallback_scale else 1000.0
    meters_per_count = 1000.0 / scale
    depth_m = np.ones((480, 640), dtype=np.float32) * 1.5  # 模拟 1.5米深度
    return depth_m, scale, meters_per_count


class Dummy3DObjectNode(Node):
    def __init__(self):
        super().__init__("dummy_mos_3d_object")
        print("【DUMMY】初始化 Dummy3DObjectNode")
        self._declare_parameters()
        self._load_parameters()

        # try:
        #     from mos_sdk import CameraType, MosSensorAdapter
        # except ImportError as exc:
        #     raise RuntimeError(
        #         "mos_sdk is unavailable. Run this node inside the Lumos/MOS SDK environment."
        #     ) from exc
        print("【DUMMY】跳过 mos_sdk 导入")

        if self.camera not in CAMERA_ENUM_NAMES:
            raise ValueError(f"unsupported camera: {self.camera}")

        # self.camera_type = getattr(CameraType, CAMERA_ENUM_NAMES[self.camera])
        # self.sensor = MosSensorAdapter(self.config)
        # if not self.sensor.start():
        #     raise RuntimeError("MosSensorAdapter.start() failed")
        print(f"【DUMMY】跳过 MosSensorAdapter，camera={self.camera}, config={self.config}")
        self.camera_type = None
        self.sensor = None

        # color_intrinsic = self.sensor.get_color_intrinsics(self.camera_type)
        # intrinsic_data = sdk_intrinsic_to_dict(color_intrinsic)
        # self.localizer = RGBDObjectLocalizer(
        #     rgb_intrinsic_data=intrinsic_data,
        #     depth_intrinsic_data=intrinsic_data,
        #     depth_is_rgb_aligned=True,
        #     min_depth_m=self.min_depth_m,
        #     max_depth_m=self.max_depth_m,
        #     model_size=self.model_size,
        #     weights=self.weights or None,
        #     conf=self.conf,
        #     iou=self.iou,
        #     device=self.device,
        #     target_classes=self.target_classes or None,
        # )
        print(f"【DUMMY】跳过 RGBDObjectLocalizer，model_size={self.model_size}, device={self.device}")
        self.localizer = None

        # self.transformer = CoordinateTransformer(
        #     self,
        #     use_tf=self.use_tf,
        #     target_frame=self.target_frame,
        #     camera_frame=self.camera_frame,
        #     camera_name=self.camera,
        #     static_extrinsic_file=self.static_extrinsic_file,
        #     invert_static_extrinsic=self.invert_static_extrinsic,
        #     tf_timeout_sec=self.tf_timeout_sec,
        #     use_latest_tf=self.use_latest_tf,
        # )
        print(f"【DUMMY】跳过 CoordinateTransformer，use_tf={self.use_tf}, target_frame={self.target_frame}")
        self.transformer = None

        self.pub_detections_3d = None
        if self.publish_detections_3d and DetectedObject3DArray is not None:
            self.pub_detections_3d = self.create_publisher(
                DetectedObject3DArray, self.detections_3d_topic, 10
            )
            print(f"【DUMMY】创建发布器: {self.detections_3d_topic}")
        elif self.publish_detections_3d:
            self.get_logger().warn(
                "DetectedObject3DArray message is unavailable; "
                "/vision/detections_3d will not be published until the package is rebuilt"
            )
        self.processed = 0
        self.srv_set_enabled = self.create_service(
            SetBool, "~/set_enabled", self._handle_set_enabled
        )
        self.timer = self.create_timer(self.poll_seconds, self._tick)
        self.get_logger().info(
            f"【DUMMY】启动 3D object 节点 camera={self.camera} use_tf={self.use_tf} "
            f"target_frame={self.target_frame} enabled={self.enabled}"
        )

    def _declare_parameters(self):
        defaults = {
            "config": "/opt/lumos/config/mos_sensor/example.yaml",
            "camera": "head",
            "weights": "",
            "model_size": "nano",
            "enabled": True,  # DUMMY: 默认启用
            "target_classes": "",
            "conf": 0.25,
            "iou": 0.45,
            "device": "cpu",
            "min_depth_m": 0.15,
            "max_depth_m": 12.0,
            "depth_scale": 0.0,
            "poll_seconds": 2.0,  # DUMMY: 2秒发一次
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

    def _tick(self):
        if not self.enabled:
            return

        frame_index = self.processed + 1
        tick_t0 = time.perf_counter()
        # 只在第1帧和每10帧打印一次
        show_details = (frame_index == 1) or (frame_index % 10 == 0)

        # get_rgbd_t0 = time.perf_counter()
        # pair = self.sensor.get_rgbd_data(self.camera_type)
        # get_rgbd_t1 = time.perf_counter()
        # if not pair:
        #     return
        # rgb, depth = pair
        # if rgb is None or depth is None:
        #     return
        if show_details:
            print(f"【DUMMY】3D检测帧 {frame_index}: camera={self.camera_type}, target_frame={self.target_frame}")
        pair = True
        rgb = None
        depth = None

        # convert_t0 = time.perf_counter()
        # image = rgb.convert_to_cv2_mat()
        # depth_raw = depth.convert_to_cv2_mat()
        # convert_t1 = time.perf_counter()
        # if image is None or depth_raw is None or image.size == 0 or depth_raw.size == 0:
        #     return
        # if image.shape[:2] != depth_raw.shape[:2]:
        #     self.get_logger().warn(
        #         f"skip unaligned frame: RGB {image.shape[:2]}, depth {depth_raw.shape[:2]}"
        #     )
        #     return
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth_raw = np.ones((480, 640), dtype=np.uint16) * 1500  # 1.5m

        # rgb_timestamp_ns = header_value(rgb, "timestamp_ns")
        # depth_timestamp_ns = header_value(depth, "timestamp_ns")
        # delta_ms = None
        # if rgb_timestamp_ns is not None and depth_timestamp_ns is not None:
        #     delta_ms = abs(int(rgb_timestamp_ns) - int(depth_timestamp_ns)) / 1e6
        #     if (
        #         self.max_rgb_depth_delta_ms is not None
        #         and delta_ms > self.max_rgb_depth_delta_ms
        #     ):
        #         self.get_logger().warn(
        #             f"skip RGB-D pair with {delta_ms:.3f} ms timestamp delta"
        #         )
        #         return
        timestamp_ns = int(time.time() * 1e9)
        delta_ms = 0.0

        # camera_frame = (
        #     self.camera_frame
        #     or header_value(rgb, "frame_id")
        #     or CAMERA_ENUM_NAMES[self.camera]
        # )
        camera_frame = self.camera_frame or CAMERA_ENUM_NAMES[self.camera]

        # try:
        #     T_target_camera, target_frame, camera_frame, transform_source = (
        #         self.transformer.resolve(camera_frame, timestamp_ns)
        #     )
        # except Exception as exc:
        #     if self.use_tf:
        #         self.get_logger().warn(f"TF lookup failed: {exc}")
        #     T_target_camera = None
        #     target_frame = self.target_frame
        #     transform_source = None
        T_target_camera = np.eye(4)
        target_frame = self.target_frame
        transform_source = "dummy_identity"

        # depth_m, used_depth_scale, depth_unit_m = depth_to_meters(
        #     depth, depth_raw, fallback_scale=self.depth_scale
        # )
        depth_m, used_depth_scale, depth_unit_m = depth_to_meters(
            None, depth_raw, fallback_scale=self.depth_scale
        )

        # records = self.localizer.localize(
        #     image,
        #     depth_m,
        #     timestamp=timestamp_ns,
        #     track=self.track,
        #     T_target_camera=T_target_camera,
        #     target_frame=target_frame,
        #     camera_frame=camera_frame,
        #     transform_source=transform_source,
        # )
        records = self._generate_dummy_detections(
            timestamp_ns, camera_frame, target_frame, transform_source
        )

        for record in records:
            # record["timestamp_ns"] = int(timestamp_ns) if timestamp_ns is not None else None
            # record["rgb_timestamp_ns"] = (
            #     int(rgb_timestamp_ns) if rgb_timestamp_ns is not None else None
            # )
            # record["depth_timestamp_ns"] = (
            #     int(depth_timestamp_ns) if depth_timestamp_ns is not None else None
            # )
            # record["rgb_depth_delta_ms"] = delta_ms
            # record["depth_frame"] = header_value(depth, "frame_id")
            # record["depth_scale"] = used_depth_scale
            # record["depth_unit_m"] = depth_unit_m
            # record["camera_type"] = CAMERA_ENUM_NAMES[self.camera]
            record["timestamp_ns"] = int(timestamp_ns)
            record["rgb_timestamp_ns"] = int(timestamp_ns)
            record["depth_timestamp_ns"] = int(timestamp_ns)
            record["rgb_depth_delta_ms"] = delta_ms
            record["depth_frame"] = camera_frame
            record["depth_scale"] = used_depth_scale
            record["depth_unit_m"] = depth_unit_m
            record["camera_type"] = CAMERA_ENUM_NAMES[self.camera]

        self._publish_detections_3d(records, camera_frame, target_frame)
        
        self.processed += 1
        tick_t1 = time.perf_counter()
        if show_details:
            print(f"【DUMMY】3D检测帧 {frame_index}: 检测 {len(records)} 目标，耗时 {(tick_t1-tick_t0)*1000:.1f}ms")

    def _generate_dummy_detections(self, timestamp_ns, camera_frame, target_frame, transform_source):
        """生成硬编码的测试检测数据"""
        # 模拟检测到一个购物车，位置在机器人前方 1.5m 处
        records = [
            {
                "class_id": 0,  # 假设 class_id 0 是购物车
                "class_name": "治疗车",
                "score": 0.85,
                "object_id": 1,
                "bbox_xyxy": [200, 150, 450, 400],  # 像素坐标
                "position_camera_m": [0.0, 0.0, 1.5],  # 相机坐标系：前方1.5m
                "position_target_m": [1.5, 0.0, 0.0],  # base_link坐标系：x轴前方1.5m
                "depth_m": 1.5,
                "camera_frame": camera_frame,
                "target_frame": target_frame,
                "transform_source": transform_source,
            }
        ]
        return records

    def _handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        state = "enabled" if self.enabled else "disabled"
        response.success = True
        response.message = f"dummy_mos_3d_object {state}"
        print(f"【DUMMY】服务调用: {response.message}")
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
        # if hasattr(self, "sensor") and self.sensor is not None:
        #     self.sensor.stop()
        print("【DUMMY】跳过 sensor.stop()")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Dummy3DObjectNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
