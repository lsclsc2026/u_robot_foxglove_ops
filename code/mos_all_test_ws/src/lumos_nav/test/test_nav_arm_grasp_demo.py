from types import SimpleNamespace

from nav_arm_grasp_demo import (
    fill_grasp_request,
    fill_home_request,
    quaternion_from_yaw,
)


def test_quaternion_from_yaw_zero_points_forward():
    qx, qy, qz, qw = quaternion_from_yaw(0.0)

    assert qx == 0.0
    assert qy == 0.0
    assert qz == 0.0
    assert qw == 1.0


def test_fill_home_request_sets_empty_points_and_home_task():
    request = SimpleNamespace(
        task_name="",
        left_point=SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            point=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        ),
        right_point=SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
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


def test_fill_grasp_request_sets_dual_arm_targets():
    request = SimpleNamespace(
        task_name="",
        left_point=SimpleNamespace(
            header=SimpleNamespace(frame_id=""),
            point=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        right_point=SimpleNamespace(
            header=SimpleNamespace(frame_id=""),
            point=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        target_frame="",
    )

    fill_grasp_request(
        request,
        left_xyz=(0.4, 0.1, 0.2),
        right_xyz=(0.4, -0.1, 0.2),
        target_frame="base_link",
    )

    assert request.task_name == "grasp"
    assert request.left_point.header.frame_id == "base_link"
    assert request.right_point.header.frame_id == "base_link"
    assert request.left_point.point.x == 0.4
    assert request.left_point.point.y == 0.1
    assert request.left_point.point.z == 0.2
    assert request.right_point.point.x == 0.4
    assert request.right_point.point.y == -0.1
    assert request.right_point.point.z == 0.2
    assert request.target_frame == "base_link"
