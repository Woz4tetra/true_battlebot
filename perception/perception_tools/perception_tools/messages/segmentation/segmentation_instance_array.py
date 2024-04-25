from __future__ import annotations

from dataclasses import dataclass, field

from perception_tools.messages.segmentation.segmentation_instance import SegmentationInstance
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class SegmentationInstanceArray:
    header: Header = field(default_factory=lambda: Header.auto())
    height: int = 0
    width: int = 0
    instances: list[SegmentationInstance] = field(default_factory=lambda: [])
    type: str = "bw_interfaces/SegmentationInstanceArray"

    def to_raw(self) -> RawRosMessage:
        pass

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> SegmentationInstanceArray:
        pass
