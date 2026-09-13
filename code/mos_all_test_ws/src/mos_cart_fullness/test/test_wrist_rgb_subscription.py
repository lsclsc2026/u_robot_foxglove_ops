import importlib
from pathlib import Path
import sys

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_latest_frame_store_keeps_a_private_snapshot():
    node_module = importlib.import_module("mos_cart_fullness.cart_fullness_node")

    assert hasattr(node_module, "LatestFrameStore")
    store = node_module.LatestFrameStore()
    frame = np.full((2, 3, 3), 7, dtype=np.uint8)
    store.update("left", frame, stamp_sec=1.0, received_sec=2.0)

    frame[:, :, :] = 1
    cached = store.peek("left")
    assert cached is not None
    assert int(cached.frame[0, 0, 0]) == 7

    cached.frame[:, :, :] = 2
    assert int(store.peek("left").frame[0, 0, 0]) == 7
    assert store.peek("right") is None


def test_cart_fullness_uses_ros_wrist_images_not_mos_sdk():
    node_source = (PACKAGE_ROOT / "mos_cart_fullness" / "cart_fullness_node.py").read_text(
        encoding="utf-8"
    )

    assert "from sensor_msgs.msg import Image" in node_source
    assert "from cv_bridge import CvBridge" in node_source
    assert "MosSensorAdapter" not in node_source
    assert "get_image_data" not in node_source
    assert '"left_rgb_topic"' in node_source
    assert '"right_rgb_topic"' in node_source
    assert '"head_rgb_topic"' in node_source


def test_wrist_topics_and_ros_dependencies_are_declared():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "cart_fullness.yaml").read_text(encoding="utf-8")
    )
    params = config["mos_cart_fullness"]["ros__parameters"]
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert params["left_rgb_topic"] == "/left_arm_camera/color/image_raw"
    assert params["right_rgb_topic"] == "/right_arm_camera/color/image_raw"
    assert params["head_rgb_topic"] == "/head_camera/color/image_raw"
    assert params["camera_poll_seconds"] == 0.10
    assert "<depend>sensor_msgs</depend>" in package_xml
    assert "<depend>cv_bridge</depend>" in package_xml
