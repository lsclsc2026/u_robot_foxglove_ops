"""Lazy YOLO wrapper for wrist-camera 2D detections."""

from __future__ import annotations

from typing import Any


MODEL_MAP = {
    "nano": "yolo11n",
    "small": "yolo11s",
    "medium": "yolo11m",
    "large": "yolo11l",
    "extra": "yolo11x",
}


class WristYoloDetector:
    """Run 2D YOLO on one RGB image and return normalized detections."""

    def __init__(
        self,
        *,
        model_size: str = "nano",
        weights: str = "",
        conf: float = 0.25,
        iou: float = 0.45,
        device: str = "cpu",
    ):
        from ultralytics import YOLO

        self.device = str(device or "cpu")
        model_name = str(weights or "") or MODEL_MAP.get(
            str(model_size).lower(),
            MODEL_MAP["nano"],
        )
        self.model = YOLO(model_name)
        self.model.overrides["conf"] = float(conf)
        self.model.overrides["iou"] = float(iou)
        self.model.overrides["agnostic_nms"] = False
        self.model.overrides["max_det"] = 1000

    def detect(self, image, *, track: bool = False) -> list[dict[str, Any]]:
        if track:
            results = self.model.track(
                image,
                verbose=False,
                device=self.device,
                persist=True,
            )
        else:
            results = self.model.predict(image, verbose=False, device=self.device)

        detections: list[dict[str, Any]] = []
        for predictions in results:
            if predictions is None or predictions.boxes is None:
                continue
            names = getattr(predictions, "names", {})
            for box in predictions.boxes:
                bbox = [float(value) for value in box.xyxy[0].cpu().numpy()]
                score = float(box.conf[0])
                class_id = int(box.cls[0])
                object_id = None
                if getattr(box, "id", None) is not None:
                    object_id = int(box.id[0])
                detections.append(
                    {
                        "class_id": class_id,
                        "class_name": class_name(names, class_id),
                        "score": score,
                        "bbox_xyxy": bbox,
                        "object_id": object_id,
                    }
                )
        return detections


def class_name(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(int(class_id), names.get(str(int(class_id)), class_id)))
    if 0 <= int(class_id) < len(names):
        return str(names[int(class_id)])
    return str(class_id)
