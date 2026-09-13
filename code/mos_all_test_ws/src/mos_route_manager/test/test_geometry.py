import pytest

from mos_route_manager.geometry import SegmentZone


def test_segment_zone_uses_xy_corridor_only():
    zone = SegmentZone("scan", 0.0, 0.0, 4.0, 0.0, 0.5)

    assert zone.contains(2.0, 0.49)
    assert not zone.contains(2.0, 0.51)
    assert not zone.contains(-0.01, 0.0)
    assert not zone.contains(4.01, 0.0)


def test_segment_zone_rejects_zero_length_segment():
    with pytest.raises(ValueError, match="non-zero"):
        SegmentZone("invalid", 1.0, 1.0, 1.0, 1.0, 0.5)
