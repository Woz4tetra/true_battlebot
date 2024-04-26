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
        return {
            "header": self.header.to_raw(),
            "height": self.height,
            "width": self.width,
            "instances": [instance.to_raw() for instance in self.instances],
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> SegmentationInstanceArray:
        return cls(
            header=Header.from_raw(msg["header"]),
            height=msg["height"],
            width=msg["width"],
            instances=[SegmentationInstance.from_raw(instance) for instance in msg["instances"]],
        )
