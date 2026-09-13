"""RGB-D YOLO localization from aligned RGB and depth frames."""

import time

import cv2
import numpy as np

from .detection_model import ObjectDetector
from .transforms import transform_point


def intrinsic_to_matrix(params):
    return np.array(
        [
            [float(params["Fx"]), 0.0, float(params["Cx"])],
            [0.0, float(params["Fy"]), float(params["Cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def intrinsic_to_distortion(params):
    return np.array(
        [
            float(params.get("k1", 0.0)),
            float(params.get("k2", 0.0)),
            float(params.get("p1", 0.0)),
            float(params.get("p2", 0.0)),
            float(params.get("k3", 0.0)),
        ],
        dtype=np.float64,
    )


def align_depth_to_rgb(
    depth_m,
    K_depth,
    dist_depth,
    K_rgb,
    dist_rgb,
    T_rgb_depth,
    rgb_shape,
    min_depth_m,
    max_depth_m,
):
    ys, xs = np.indices(depth_m.shape[:2])
    z = depth_m.reshape(-1)
    xs = xs.reshape(-1).astype(np.float64)
    ys = ys.reshape(-1).astype(np.float64)
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if not np.any(valid):
        return np.zeros(rgb_shape[:2], dtype=np.float32)

    z = z[valid].astype(np.float64)
    xs = xs[valid]
    ys = ys[valid]

    if dist_depth is not None and np.any(np.abs(dist_depth) > 0):
        pixels = np.stack([xs, ys], axis=1).reshape(-1, 1, 2)
        normalized = cv2.undistortPoints(pixels, K_depth, dist_depth).reshape(-1, 2)
        x_n = normalized[:, 0]
        y_n = normalized[:, 1]
    else:
        x_n = (xs - K_depth[0, 2]) / K_depth[0, 0]
        y_n = (ys - K_depth[1, 2]) / K_depth[1, 1]

    points_depth = np.stack([x_n * z, y_n * z, z], axis=1)
    points_rgb = (T_rgb_depth[:3, :3] @ points_depth.T).T + T_rgb_depth[:3, 3]
    in_front = points_rgb[:, 2] > min_depth_m
    points_rgb = points_rgb[in_front]
    if points_rgb.size == 0:
        return np.zeros(rgb_shape[:2], dtype=np.float32)

    image_points, _ = cv2.projectPoints(
        points_rgb,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        K_rgb,
        dist_rgb,
    )
    uv = image_points.reshape(-1, 2)
    u = np.rint(uv[:, 0]).astype(np.int32)
    v = np.rint(uv[:, 1]).astype(np.int32)

    rgb_h, rgb_w = rgb_shape[:2]
    inside = (u >= 0) & (u < rgb_w) & (v >= 0) & (v < rgb_h)
    u = u[inside]
    v = v[inside]
    z_rgb = points_rgb[inside, 2].astype(np.float32)

    aligned = np.zeros((rgb_h, rgb_w), dtype=np.float32)
    order = np.argsort(z_rgb)[::-1]
    aligned[v[order], u[order]] = z_rgb[order]
    return aligned


def robust_depth_in_bbox(depth_m, bbox, min_depth_m, max_depth_m, inner_ratio=0.6):
    h, w = depth_m.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox]
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    bw = max(1.0, x2 - x1) * inner_ratio
    bh = max(1.0, y2 - y1) * inner_ratio

    ix1 = max(0, int(round(cx - bw * 0.5)))
    iy1 = max(0, int(round(cy - bh * 0.5)))
    ix2 = min(w, int(round(cx + bw * 0.5)))
    iy2 = min(h, int(round(cy + bh * 0.5)))
    region = depth_m[iy1:iy2, ix1:ix2]
    if region.size == 0:
        return None

    valid = region[np.isfinite(region) & (region >= min_depth_m) & (region <= max_depth_m)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def backproject_pixel_to_camera(u, v, z_m, K, dist_coeffs=None):
    if dist_coeffs is not None and np.any(np.abs(dist_coeffs) > 0):
        pixel = np.array([[[float(u), float(v)]]], dtype=np.float64)
        normalized = cv2.undistortPoints(pixel, K, dist_coeffs)
        x_n, y_n = normalized.reshape(2)
        return np.array([x_n * z_m, y_n * z_m, z_m], dtype=np.float64)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    return np.array(
        [((float(u) - cx) / fx) * z_m, ((float(v) - cy) / fy) * z_m, z_m],
        dtype=np.float64,
    )


def class_name_from_model_names(model_names, class_id):
    if isinstance(model_names, dict):
        return model_names.get(int(class_id), model_names.get(str(int(class_id)), str(class_id)))
    if 0 <= int(class_id) < len(model_names):
        return model_names[int(class_id)]
    return str(class_id)


def normalize_target_classes(target_classes):
    if target_classes is None:
        return set()
    if isinstance(target_classes, str):
        target_classes = target_classes.split(",")
    return {str(item).strip().lower() for item in target_classes if str(item).strip()}


def is_target_class(target_classes, class_id, class_name):
    if not target_classes:
        return True
    return str(int(class_id)) in target_classes or class_name.lower() in target_classes


class RGBDObjectLocalizer:
    """Localize detected objects in the RGB camera optical frame."""

    def __init__(
        self,
        rgb_intrinsic_data,
        depth_intrinsic_data=None,
        T_rgb_depth=None,
        depth_is_rgb_aligned=True,
        min_depth_m=0.15,
        max_depth_m=12.0,
        model_size="nano",
        weights=None,
        conf=0.25,
        iou=0.45,
        device="cpu",
        target_classes=None,
    ):
        if depth_intrinsic_data is None:
            depth_intrinsic_data = rgb_intrinsic_data
        self.depth_is_rgb_aligned = depth_is_rgb_aligned
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.target_classes = normalize_target_classes(target_classes)

        self.K_rgb = intrinsic_to_matrix(rgb_intrinsic_data)
        self.dist_rgb = intrinsic_to_distortion(rgb_intrinsic_data)
        self.K_depth = intrinsic_to_matrix(depth_intrinsic_data)
        self.dist_depth = intrinsic_to_distortion(depth_intrinsic_data)
        self.T_rgb_depth = (
            np.asarray(T_rgb_depth, dtype=np.float64).reshape(4, 4)
            if T_rgb_depth is not None
            else np.eye(4, dtype=np.float64)
        )

        self.detector = ObjectDetector(
            model_size=model_size,
            weights=weights,
            conf_thres=conf,
            iou_thres=iou,
            device=device,
        )
        self.class_names = self.detector.get_class_names()

    def depth_to_rgb_frame(self, raw_depth_m, rgb_shape):
        if self.depth_is_rgb_aligned:
            return raw_depth_m
        return align_depth_to_rgb(
            raw_depth_m,
            self.K_depth,
            self.dist_depth,
            self.K_rgb,
            self.dist_rgb,
            self.T_rgb_depth,
            rgb_shape,
            self.min_depth_m,
            self.max_depth_m,
        )

    def localize(
        self,
        image,
        raw_depth_m,
        timestamp=None,
        track=False,
        T_target_camera=None,
        target_frame=None,
        camera_frame=None,
        transform_source=None,
        return_debug_image=False,
    ):
        localize_t0 = time.perf_counter()
        print("TIMING rgbd_localizer stage=depth_align event=start", flush=True)
        depth_m = self.depth_to_rgb_frame(raw_depth_m, image.shape)
        depth_align_t1 = time.perf_counter()
        print(
            f"TIMING rgbd_localizer stage=depth_align event=end "
            f"duration_ms={(depth_align_t1 - localize_t0) * 1000.0:.1f}",
            flush=True,
        )

        yolo_t0 = time.perf_counter()
        print("TIMING rgbd_localizer stage=yolo_detect event=start", flush=True)
        annotated_image, detections = self.detector.detect(image, track=track)
        yolo_t1 = time.perf_counter()
        print(
            f"TIMING rgbd_localizer stage=yolo_detect event=end "
            f"duration_ms={(yolo_t1 - yolo_t0) * 1000.0:.1f} "
            f"raw_detections={len(detections)}",
            flush=True,
        )

        to3d_t0 = time.perf_counter()
        print("TIMING rgbd_localizer stage=2d_to_3d event=start", flush=True)
        records = []

        for bbox, score, class_id, object_id in detections:
            class_name = class_name_from_model_names(self.class_names, class_id)
            if not is_target_class(self.target_classes, class_id, class_name):
                continue

            depth_value_m = robust_depth_in_bbox(
                depth_m, bbox, self.min_depth_m, self.max_depth_m
            )
            if depth_value_m is None:
                continue

            u = (bbox[0] + bbox[2]) * 0.5
            v = (bbox[1] + bbox[3]) * 0.5
            p_camera = backproject_pixel_to_camera(
                u, v, depth_value_m, self.K_rgb, self.dist_rgb
            )
            record = {
                "timestamp": str(timestamp) if timestamp is not None else None,
                "class_id": int(class_id),
                "class_name": class_name,
                "score": float(score),
                "object_id": int(object_id) if object_id is not None else None,
                "bbox_xyxy": [float(value) for value in bbox],
                "depth_m": depth_value_m,
                "position_camera_m": p_camera.tolist(),
                "camera_frame": camera_frame,
            }

            if T_target_camera is not None:
                p_target = transform_point(T_target_camera, p_camera)
                record.update(
                    {
                        "position_target_m": p_target.tolist(),
                        "target_frame": target_frame,
                        "target_t_camera_source": transform_source,
                        "distance_from_target_origin_m": float(np.linalg.norm(p_target)),
                    }
                )
                if str(target_frame).lower() in {"base", "base_link", "base_footprint"}:
                    record["position_base_m"] = p_target.tolist()

            records.append(record)

        to3d_t1 = time.perf_counter()
        print(
            f"TIMING rgbd_localizer stage=2d_to_3d event=end "
            f"duration_ms={(to3d_t1 - to3d_t0) * 1000.0:.1f} "
            f"raw_detections={len(detections)} target_records={len(records)}",
            flush=True,
        )
        print(
            f"TIMING rgbd_localizer_summary "
            f"depth_align_ms={(depth_align_t1 - localize_t0) * 1000.0:.1f} "
            f"yolo_detect_ms={(yolo_t1 - yolo_t0) * 1000.0:.1f} "
            f"2d_to_3d_ms={(to3d_t1 - to3d_t0) * 1000.0:.1f} "
            f"total_ms={(to3d_t1 - localize_t0) * 1000.0:.1f} "
            f"raw_detections={len(detections)} target_records={len(records)}",
            flush=True,
        )

        if return_debug_image:
            return records, annotated_image
        return records
