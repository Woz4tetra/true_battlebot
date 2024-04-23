from dataclasses import dataclass

from perception_tools.messages.segmentation.contour import Contour


@dataclass
class SegmentationInstance:
    contours: list[Contour]
    score: float
    label: str
    class_index: int
    object_index: int
    has_holes: bool
