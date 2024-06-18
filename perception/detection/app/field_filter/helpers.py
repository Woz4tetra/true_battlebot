from bw_interfaces.msg import SegmentationInstance, SegmentationInstanceArray
from bw_shared.enums.label import Label


def get_field(segmentations: SegmentationInstanceArray) -> SegmentationInstance:
    fields = [seg for seg in segmentations.instances if seg.label == Label.FIELD]
    if len(fields) > 1:
        raise ValueError("Multiple field segmentations detected, using the first one")
    elif len(fields) == 0:
        raise ValueError("No field segmentation detected")
    field = fields[0]
    return field
