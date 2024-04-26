from __future__ import annotations

from dataclasses import dataclass

from perception_tools.messages.segmentation.contour import Contour
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class SegmentationInstance:
    contours: list[Contour]
    score: float
    label: str
    class_index: int
    object_index: int
    has_holes: bool

    def to_raw(self) -> RawRosMessage:
        return {
            "contours": [contour.to_raw() for contour in self.contours],
            "score": self.score,
            "label": self.label,
            "class_index": self.class_index,
            "object_index": self.object_index,
            "has_holes": self.has_holes,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> SegmentationInstance:
        return cls(
            contours=[Contour.from_raw(contour) for contour in msg["contours"]],
            score=msg["score"],
            label=msg["label"],
            class_index=msg["class_index"],
            object_index=msg["object_index"],
            has_holes=msg["has_holes"],
        )
