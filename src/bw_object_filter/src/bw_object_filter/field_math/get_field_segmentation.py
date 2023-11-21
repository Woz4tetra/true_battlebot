from bw_interfaces.msg import SegmentationInstance, SegmentationInstanceArray


def get_field_segmentation(segmentations: SegmentationInstanceArray) -> SegmentationInstance:
    field_instance = SegmentationInstance()
    largest_area = 0
    for instance in segmentations.instances:
        if instance.label == "field":
            area = sum([c.area for c in instance.contours])
            if area > largest_area:
                field_instance = instance
                largest_area = area
    return field_instance
