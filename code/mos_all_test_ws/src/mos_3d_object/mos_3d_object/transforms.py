"""Rigid transform helpers for camera-to-robot point conversion."""

import json
from pathlib import Path

import numpy as np


def quat_to_rotation_matrix(q):
    """Convert quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    w, x, y, z = [float(value) for value in q]
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_from_row_major(values):
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError(f"expected 16 matrix values, got {matrix.size}")
    return matrix.reshape(4, 4)


def validate_rigid_transform(matrix, name="transform"):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{name} last row must be [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    orthogonal_error = np.linalg.norm(rotation.T @ rotation - np.eye(3))
    determinant = np.linalg.det(rotation)
    if orthogonal_error > 1e-3 or abs(determinant - 1.0) > 1e-3:
        raise ValueError(
            f"{name} rotation is invalid: error={orthogonal_error:.6g}, "
            f"det={determinant:.6g}"
        )
    return matrix


def load_structured_file(path):
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in {".yaml", ".yml"}:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for YAML static extrinsics") from exc
        return yaml.safe_load(text)
    return json.loads(text)


def transform_from_data(data):
    if isinstance(data, list) and len(data) == 16:
        return matrix_from_row_major(data)
    if isinstance(data, list):
        if not data:
            raise ValueError("empty transform list")
        data = data[0]
    if "static_fallback_matrix" in data:
        return matrix_from_row_major(data["static_fallback_matrix"])
    if "matrix" in data:
        return matrix_from_row_major(data["matrix"])
    if "static_base_t_camera" in data:
        return transform_from_data(data["static_base_t_camera"])
    if "static_target_t_camera" in data:
        return transform_from_data(data["static_target_t_camera"])

    rotation = data["rotation"]
    if isinstance(rotation, dict):
        q = [rotation["w"], rotation["x"], rotation["y"], rotation["z"]]
        R = quat_to_rotation_matrix(q)
    else:
        R = np.asarray(rotation, dtype=np.float64).reshape(3, 3)

    translation = data["translation"]
    if isinstance(translation, dict):
        t = np.array([translation["x"], translation["y"], translation["z"]], dtype=np.float64)
    else:
        t = np.asarray(translation, dtype=np.float64).reshape(3)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def select_transform_data(data, camera_name=None, source_frame=None):
    """Select one transform entry from a single-entry or multi-camera file."""
    if not isinstance(data, dict):
        return data, None

    camera_name = str(camera_name or "")
    source_frame = str(source_frame or "")

    if camera_name and camera_name in data and isinstance(data[camera_name], dict):
        return data[camera_name], camera_name

    entries = None
    if isinstance(data.get("transforms"), list):
        entries = data["transforms"]
    elif isinstance(data.get("transforms"), dict):
        entries = [
            {"camera": key, **value} if isinstance(value, dict) else value
            for key, value in data["transforms"].items()
        ]
    elif isinstance(data.get("cameras"), dict):
        entries = [
            {"camera": key, **value} if isinstance(value, dict) else value
            for key, value in data["cameras"].items()
        ]

    if entries is None:
        return data, None

    if camera_name:
        for entry in entries:
            if isinstance(entry, dict) and str(entry.get("camera", "")) == camera_name:
                return entry, camera_name

    if source_frame:
        for entry in entries:
            if isinstance(entry, dict) and str(entry.get("source_frame", "")) == source_frame:
                return entry, source_frame

    if len(entries) == 1:
        return entries[0], None

    raise KeyError(
        f"no static transform for camera={camera_name!r} source_frame={source_frame!r}"
    )


def load_transform_entry(path, camera_name=None, source_frame=None, invert=False):
    data = load_structured_file(path)
    entry, key = select_transform_data(
        data, camera_name=camera_name, source_frame=source_frame
    )
    T = validate_rigid_transform(transform_from_data(entry), str(path))
    target_frame = entry.get("target_frame") if isinstance(entry, dict) else None
    selected_source_frame = entry.get("source_frame") if isinstance(entry, dict) else None
    if invert:
        return np.linalg.inv(T), selected_source_frame, target_frame, key
    return T, target_frame, selected_source_frame, key


def load_transform_file(path, invert=False):
    T, _, _, _ = load_transform_entry(path, invert=invert)
    return T


def transform_point(T_target_source, p_source):
    p_h = np.array([p_source[0], p_source[1], p_source[2], 1.0], dtype=np.float64)
    return (np.asarray(T_target_source, dtype=np.float64) @ p_h)[:3]


def transform_stamped_to_matrix(transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rotation_matrix([q.w, q.x, q.y, q.z])
    T[:3, 3] = [t.x, t.y, t.z]
    return validate_rigid_transform(T, "tf transform")
