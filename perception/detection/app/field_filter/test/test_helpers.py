import os

print(os.listdir("."))
import pytest
from app.field_filter.helpers import get_field
from bw_interfaces.msg import SegmentationInstance, SegmentationInstanceArray


def test_get_field() -> None:
    field = SegmentationInstance(label="field")
    other_field = SegmentationInstance(label="field")
    robot1 = SegmentationInstance(label="robot1")
    robot2 = SegmentationInstance(label="robot2")

    assert get_field(SegmentationInstanceArray(instances=[field])) == field
    assert get_field(SegmentationInstanceArray(instances=[field, robot1, robot2])) == field

    with pytest.raises(ValueError):
        get_field(SegmentationInstanceArray(instances=[]))

    with pytest.raises(ValueError):
        get_field(SegmentationInstanceArray(instances=[robot1, robot2]))

    with pytest.raises(ValueError):
        get_field(SegmentationInstanceArray(instances=[field, other_field, robot1, robot2]))
