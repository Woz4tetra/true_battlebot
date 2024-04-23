from dataclasses import dataclass, field

from perception_tools.messages.segmentation.segmentation_instance import SegmentationInstance
from perception_tools.messages.std_msgs.header import Header


@dataclass
class SegmentationInstanceArray:
    header: Header = Header.auto()
    height: int = 0
    width: int = 0
    instances: list[SegmentationInstance] = field(default_factory=lambda: [])
