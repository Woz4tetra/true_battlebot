from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from bw_shared.enums.label import ModelLabel


class YoloVisibility(IntEnum):
    NOT_LABELED = 0
    LABELED_NOT_VISIBLE = 1
    LABELED_VISIBLE = 2


@dataclass
class YoloKeypointAnnotation:
    class_index: int = -1
    bbox: list[float] = field(default_factory=lambda: [0, 0, 0, 0])
    keypoints: list[tuple[float, float, YoloVisibility]] = field(default_factory=list)

    def __post_init__(self):
        if len(self.bbox) != 4:
            raise ValueError("bbox must have 4 elements")

    @property
    def center_x(self) -> float:
        return self.bbox[0]

    @center_x.setter
    def center_x(self, value: float):
        self.bbox[0] = value

    @property
    def center_y(self) -> float:
        return self.bbox[1]

    @center_y.setter
    def center_y(self, value: float):
        self.bbox[1] = value

    @property
    def width(self) -> float:
        return self.bbox[2]

    @width.setter
    def width(self, value: float):
        self.bbox[2] = value

    @property
    def height(self) -> float:
        return self.bbox[3]

    @height.setter
    def height(self, value: float):
        self.bbox[3] = value

    @property
    def x0(self) -> float:
        return self.center_x - self.width / 2

    @property
    def y0(self) -> float:
        return self.center_y - self.height / 2

    @property
    def x1(self) -> float:
        return self.center_x + self.width / 2

    @property
    def y1(self) -> float:
        return self.center_y + self.height / 2

    @classmethod
    def from_corners(
        cls, x0: float, y0: float, x1: float, y1: float, class_index: int, keypoints: list | None = None
    ) -> YoloKeypointAnnotation:
        keypoints = keypoints or []
        self = cls(class_index=class_index, keypoints=keypoints)
        self.center_x = (x0 + x1) / 2
        self.center_y = (y0 + y1) / 2
        self.width = x1 - x0
        self.height = y1 - y0
        return self

    @classmethod
    def from_xywh(
        cls, x: float, y: float, w: float, h: float, class_index: int, keypoints: list | None = None
    ) -> YoloKeypointAnnotation:
        keypoints = keypoints or []
        self = cls(class_index=class_index, keypoints=keypoints)
        self.center_x = x
        self.center_y = y
        self.width = w
        self.height = h
        return self

    def to_row(self) -> str:
        label = str(self.class_index) + " "
        label += " ".join(str(x) for x in self.bbox)
        for keypoint in self.keypoints:
            label += " " + " ".join(str(x) for x in keypoint)
        label += "\n"
        return label


@dataclass
class YoloKeypointImage:
    image_id: str = ""
    labels: list[YoloKeypointAnnotation] = field(default_factory=list)

    def to_txt(self) -> str:
        return "".join(label.to_row() for label in self.labels)


@dataclass
class YoloKeypointDataset:
    kpt_shape: tuple[int, int] = (2, 3)
    names: list[ModelLabel] = field(default_factory=list)
