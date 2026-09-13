#!/usr/bin/env python3
"""Bridge FAST-LIO localization outputs into a Nav2-compatible TF chain."""
import math
import numpy as np

# Keep legacy tf_transformations / transforms3d working with NumPy >= 1.24.
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import (
    translation_from_matrix,
    quaternion_from_matrix,
    translation_matrix,
    quaternion_matrix,
)

def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _odom_to_mat(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return np.matmul(
        translation_matrix([p.x, p.y, p.z]),
        quaternion_matrix([q.x, q.y, q.z, q.w]),
    )
def _mat_to_transform(mat, stamp, parent, child):
    xyz = translation_from_matrix(mat)
    quat = quaternion_from_matrix(mat)
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = xyz[0]
    t.transform.translation.y = xyz[1]
    t.transform.translation.z = xyz[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


def _identity_transform(stamp, parent, child):
    return _mat_to_transform(np.identity(4), stamp, parent, child)
class LocalizationTfBridge(Node):
    def __init__(self):
        super().__init__('localization_tf_bridge')
        self._odom = None
        self._localization = None
        self._publish_count = 0
        self._map_frame = 'map'
        self._odom_frame = 'camera_init'
        self._base_frame = 'base_link'
        self._sensor_frame = 'livox_frame'
        self._legacy_body_frame = 'body'
        self.declare_parameter('livox_to_base.x', -0.061)
        self.declare_parameter('livox_to_base.y', 0.0)
        self.declare_parameter('livox_to_base.z', 0.0)
        self.declare_parameter('livox_to_base.qx', 1.0)
        self.declare_parameter('livox_to_base.qy', 0.0)
        self.declare_parameter('livox_to_base.qz', 0.0)
        self.declare_parameter('livox_to_base.qw', 0.0)
        self._sensor_to_base = self._sensor_to_base_from_params()
        self._base_to_sensor = np.linalg.inv(self._sensor_to_base)
        from tf2_ros import TransformBroadcaster

        self.br = TransformBroadcaster(self)

        self._prev_loc_stamp = None
        self._prev_loc_pose = None
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self._sub_odom = self.create_subscription(
            Odometry, '/Odometry', self._cb_odom, 10)
        self._sub_localization = self.create_subscription(
            Odometry, '/localization', self._cb_localization, 10)
        self._timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(
            'localization_tf_bridge started '
            '(rebuilding map->camera_init from /localization and /Odometry, plus camera_init->base_link and base_link->body)')

    def _sensor_to_base_from_params(self):
        translation = [
            float(self.get_parameter('livox_to_base.x').value),
            float(self.get_parameter('livox_to_base.y').value),
            float(self.get_parameter('livox_to_base.z').value),
        ]
        quaternion = [
            float(self.get_parameter('livox_to_base.qx').value),
            float(self.get_parameter('livox_to_base.qy').value),
            float(self.get_parameter('livox_to_base.qz').value),
            float(self.get_parameter('livox_to_base.qw').value),
        ]
        return np.matmul(
            translation_matrix(translation),
            quaternion_matrix(quaternion),
        )

    def _cb_odom(self, msg):
        self._odom = msg
        self.get_logger().debug('received /Odometry')

    def _cb_localization(self, msg):
        self._localization = msg
        self._publish_odometry(msg)
        self.get_logger().debug('received /localization')

    def _publish_odometry(self, msg):
        pose_mat = _odom_to_mat(msg)
        stamp_s = _stamp_to_sec(msg.header.stamp)

        if self._prev_loc_pose is not None and self._prev_loc_stamp is not None:
            dt = stamp_s - self._prev_loc_stamp
            if dt > 0.001:
                T_rel = np.matmul(np.linalg.inv(self._prev_loc_pose), pose_mat)
                dx = T_rel[0, 3]
                dy = T_rel[1, 3]
                yaw = math.atan2(T_rel[1, 0], T_rel[0, 0])

                odom = Odometry()
                odom.header.stamp = msg.header.stamp
                odom.header.frame_id = self._map_frame
                odom.child_frame_id = self._base_frame
                odom.pose.pose = msg.pose.pose
                odom.twist.twist.linear.x = math.sqrt(dx * dx + dy * dy) / dt
                odom.twist.twist.angular.z = yaw / dt
                self._odom_pub.publish(odom)

        self._prev_loc_pose = pose_mat
        self._prev_loc_stamp = stamp_s

    def _camera_to_sensor(self):
        if self._odom is None:
            return None

        odom_parent = self._odom.header.frame_id
        odom_child = self._odom.child_frame_id
        T_parent_to_child = _odom_to_mat(self._odom)

        if odom_parent != self._odom_frame:
            self.get_logger().warn(
                f'unexpected /Odometry parent frame {odom_parent!r}, expected {self._odom_frame!r}',
                throttle_duration_sec=5.0,
            )

        if odom_child == self._sensor_frame:
            return T_parent_to_child

        if odom_child == self._base_frame:
            return np.matmul(T_parent_to_child, self._base_to_sensor)

        if odom_child == self._legacy_body_frame:
            return np.matmul(T_parent_to_child, self._base_to_sensor)

        self.get_logger().warn(
            f'unexpected /Odometry child frame {odom_child!r}',
            throttle_duration_sec=5.0,
        )
        return None

    def _camera_to_base(self, T_camera_to_sensor):
        if T_camera_to_sensor is None:
            return None
        return np.matmul(T_camera_to_sensor, self._sensor_to_base)

    def _map_to_camera(self, T_camera_to_sensor, T_camera_to_base):
        if self._localization is None:
            return None

        map_parent = self._localization.header.frame_id
        map_child = self._localization.child_frame_id
        T_map_to_child = _odom_to_mat(self._localization)

        if map_parent != self._map_frame:
            self.get_logger().warn(
                f'unexpected /localization parent frame {map_parent!r}, expected {self._map_frame!r}',
                throttle_duration_sec=5.0,
            )

        if map_child == self._odom_frame:
            return T_map_to_child

        if map_child == self._sensor_frame:
            if T_camera_to_sensor is None:
                return None
            return np.matmul(T_map_to_child, np.linalg.inv(T_camera_to_sensor))

        if map_child in (self._base_frame, self._legacy_body_frame):
            if T_camera_to_base is None:
                return None
            return np.matmul(T_map_to_child, np.linalg.inv(T_camera_to_base))

        self.get_logger().warn(
            f'unexpected /localization child frame {map_child!r}',
            throttle_duration_sec=5.0,
        )
        return None

    def _publish(self):
        T_camera_to_sensor = self._camera_to_sensor()
        T_camera_to_base = self._camera_to_base(T_camera_to_sensor) if T_camera_to_sensor is not None else None

        stamp = self.get_clock().now().to_msg()
        transforms = []

        if T_camera_to_sensor is not None:
            transforms.append(
                _mat_to_transform(
                    T_camera_to_sensor,
                    stamp,
                    self._odom_frame,
                    self._sensor_frame,
                )
            )
        else:
            self.get_logger().warn(
                '/Odometry not yet received, cannot publish TF chain',
                throttle_duration_sec=5.0,
            )

        if T_camera_to_base is not None:
            transforms.append(
                _mat_to_transform(
                    T_camera_to_base,
                    stamp,
                    self._odom_frame,
                    self._base_frame,
                )
            )

        if self._legacy_body_frame != self._base_frame:
            transforms.append(
                _identity_transform(stamp, self._base_frame, self._legacy_body_frame)
            )

        if transforms:
            self.br.sendTransform(transforms)
            self._publish_count += 1
            if self._publish_count % 100 == 0:
                self.get_logger().info(
                    f'published {self._publish_count} TF updates')


def main():
    rclpy.init()
    node = LocalizationTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
