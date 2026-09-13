"""Coordinate transformation backend using ROS TF or a static matrix."""

import numpy as np

from .transforms import load_transform_entry, transform_stamped_to_matrix


class CoordinateTransformer:
    def __init__(
        self,
        node,
        use_tf=True,
        target_frame="base_link",
        camera_frame="",
        camera_name="",
        static_extrinsic_file="",
        invert_static_extrinsic=False,
        tf_timeout_sec=0.1,
        use_latest_tf=True,
    ):
        self.node = node
        self.use_tf = bool(use_tf)
        self.target_frame = str(target_frame)
        self.camera_frame = str(camera_frame or "")
        self.camera_name = str(camera_name or "")
        self.static_extrinsic_file = str(static_extrinsic_file or "")
        self.tf_timeout_sec = float(tf_timeout_sec)
        self.use_latest_tf = bool(use_latest_tf)
        self.static_T_target_camera = None

        self.tf_buffer = None
        self.tf_listener = None
        if self.use_tf:
            import tf2_ros

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        else:
            if not self.static_extrinsic_file:
                raise ValueError("static_extrinsic_file is required when use_tf is false")
            (
                self.static_T_target_camera,
                static_target_frame,
                static_source_frame,
                self.static_transform_key,
            ) = load_transform_entry(
                self.static_extrinsic_file,
                camera_name=self.camera_name,
                source_frame=self.camera_frame,
                invert=invert_static_extrinsic,
            )
            if static_target_frame:
                self.target_frame = str(static_target_frame)
            if static_source_frame:
                self.camera_frame = str(static_source_frame)

    def resolve(self, source_frame=None, timestamp_ns=None):
        source_frame = str(self.camera_frame or source_frame or "")
        if not source_frame:
            raise ValueError("camera/source frame is empty")

        if not self.use_tf:
            return (
                np.asarray(self.static_T_target_camera, dtype=np.float64),
                self.target_frame,
                source_frame,
                f"{self.static_extrinsic_file}#{self.static_transform_key or source_frame}",
            )

        import rclpy.duration
        import rclpy.time

        if self.use_latest_tf or timestamp_ns is None:
            stamp = rclpy.time.Time()
        else:
            stamp = rclpy.time.Time(nanoseconds=int(timestamp_ns))

        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
        )
        return (
            transform_stamped_to_matrix(transform),
            transform.header.frame_id,
            transform.child_frame_id,
            "tf",
        )
