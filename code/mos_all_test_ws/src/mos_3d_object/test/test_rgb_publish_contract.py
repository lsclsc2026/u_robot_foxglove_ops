from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_node_explicitly_publishes_all_three_rgb_streams():
    source = (PACKAGE_ROOT / "mos_3d_object" / "vision_node.py").read_text(
        encoding="utf-8"
    )

    assert "from sensor_msgs.msg import Image" in source
    assert '"/head_camera/color/image_raw"' in source
    assert '"/left_arm_camera/color/image_raw"' in source
    assert '"/right_arm_camera/color/image_raw"' in source
    assert "self._publish_rgb_frames" in source
    assert "self.sensor.get_image_data" in source


def test_rgb_publish_rate_is_configured_separately_from_head_3d_rate():
    config = (PACKAGE_ROOT / "config" / "yolo_params.yaml").read_text(
        encoding="utf-8"
    )

    assert "rgb_publish_seconds:" in config
    assert "head_rgb_topic:" in config
    assert "left_rgb_topic:" in config
    assert "right_rgb_topic:" in config
