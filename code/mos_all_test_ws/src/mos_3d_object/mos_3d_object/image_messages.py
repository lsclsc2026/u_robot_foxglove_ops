"""Small ROS Image message helpers for MOS camera frames."""

import numpy as np


def bgr_to_image_message(frame, *, stamp, frame_id, image_type):
    """Build a ``sensor_msgs/Image`` compatible BGR8 message from a cv2 frame."""
    image = np.asarray(frame)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 BGR frame")
    if image.dtype != np.uint8:
        image = image.astype(np.uint8, copy=False)
    image = np.ascontiguousarray(image)

    message = image_type()
    message.header.stamp = stamp
    message.header.frame_id = str(frame_id or "")
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(image.shape[1] * image.shape[2])
    message.data = image.tobytes()
    return message
