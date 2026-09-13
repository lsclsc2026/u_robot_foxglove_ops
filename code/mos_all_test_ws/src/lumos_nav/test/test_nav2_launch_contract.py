from pathlib import Path


def test_fastlio_bringup_disables_direct_base_controller_by_default():
    source = (
        Path(__file__).parents[1] / "launch" / "nav2_fastlio_bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert "enable_direct_base_controller" in source
    assert "default_value='false'" in source
    assert "condition=IfCondition(enable_direct_base_controller)" in source
