from types import SimpleNamespace

import numpy as np


def test_bgr_frame_becomes_a_ros_image_message():
    from mos_3d_object.image_messages import bgr_to_image_message

    frame = np.array(
        [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ],
        dtype=np.uint8,
    )
    stamp = SimpleNamespace(sec=12, nanosec=345)

    message = bgr_to_image_message(
        frame,
        stamp=stamp,
        frame_id="left_arm_camera_optical_frame",
        image_type=FakeImage,
    )

    assert message.header.stamp is stamp
    assert message.header.frame_id == "left_arm_camera_optical_frame"
    assert message.height == 2
    assert message.width == 2
    assert message.encoding == "bgr8"
    assert message.is_bigendian == 0
    assert message.step == 6
    assert message.data == frame.tobytes()


def test_bgr_frame_rejects_invalid_shape():
    from mos_3d_object.image_messages import bgr_to_image_message

    with np.testing.assert_raises_regex(ValueError, "HxWx3"):
        bgr_to_image_message(
            np.zeros((2, 2), dtype=np.uint8),
            stamp=SimpleNamespace(),
            frame_id="camera",
            image_type=FakeImage,
        )


class FakeImage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""
