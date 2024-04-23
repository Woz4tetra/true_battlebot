from typing import Optional

from bw_interfaces.msg import SegmentationInstance, SegmentationInstanceArray

from bw_tools.structs.labels import Label


def get_field_segmentation(segmentations: SegmentationInstanceArray) -> Optional[SegmentationInstance]:
    field_instance = SegmentationInstance()
    largest_area = 0
    for instance in segmentations.instances:
        try:
            label = Label(instance.label)
        except ValueError:
            continue
        if label == Label.FIELD:
            area = sum([c.area for c in instance.contours])
            if area > largest_area:
                field_instance = instance
                largest_area = area
    if largest_area == 0:
        return None
    return field_instance
