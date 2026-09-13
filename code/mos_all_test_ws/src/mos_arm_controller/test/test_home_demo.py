from pathlib import Path
from types import SimpleNamespace

from mos_arm_controller.home_demo import fill_home_request


def test_fill_home_request_sets_home_task_and_empty_targets():
    request = SimpleNamespace(
        task_name="",
        left_point=SimpleNamespace(
            header=SimpleNamespace(frame_id="old"),
            point=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        ),
        right_point=SimpleNamespace(
            header=SimpleNamespace(frame_id="old"),
            point=SimpleNamespace(x=4.0, y=5.0, z=6.0),
        ),
        target_frame="map",
    )

    fill_home_request(request)

    assert request.task_name == "home"
    assert request.left_point.header.frame_id == ""
    assert request.right_point.header.frame_id == ""
    assert request.left_point.point.x == 0.0
    assert request.left_point.point.y == 0.0
    assert request.left_point.point.z == 0.0
    assert request.right_point.point.x == 0.0
    assert request.right_point.point.y == 0.0
    assert request.right_point.point.z == 0.0
    assert request.target_frame == ""


def test_hardware_controller_source_supports_home_task():
    source = (
        Path(__file__).parents[1] / "mos_arm_controller" / "hardware_controller.py"
    ).read_text(encoding="utf-8")

    assert 'SUPPORTED_TASKS = ("grasp", "place", "elevator", "home")' in source
    assert 'elif task_name == "home":' in source
    assert "def _do_home(self):" in source
    assert "self._controller.go_home(ARM_ID.BOTH_ARM)" in source
