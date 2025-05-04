from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from bw_shared.enums.label import ModelLabel
from bw_shared.messages.dataclass_utils import to_dict


class YoloVisibility(IntEnum):
    NOT_LABELED = 0
    LABELED_NOT_VISIBLE = 1
    LABELED_VISIBLE = 2


YoloKeypointData = tuple[float, float, YoloVisibility]


@dataclass(eq=False)
class YoloKeypointAnnotation:
    class_index: int = -1
    bbox: list[float] = field(default_factory=lambda: [0, 0, 0, 0])
    keypoints: list[YoloKeypointData] = field(default_factory=list)

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
        self.center_x = x + w / 2
        self.center_y = y + h / 2
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

    @classmethod
    def from_row(cls, row: str) -> YoloKeypointAnnotation:
        parts = row.split()
        class_index = int(parts[0])
        bbox = [float(x) for x in parts[1:5]]
        keypoints: list[tuple[float, float, YoloVisibility]] = []
        for i in range(5, len(parts), 3):
            x_keypoint = float(parts[i])
            y_keypoint = float(parts[i + 1])
            visibility = YoloVisibility(int(parts[i + 2]))
            keypoints.append((x_keypoint, y_keypoint, visibility))
        return cls(class_index, bbox, keypoints)

    def __hash__(self) -> int:
        hash_value = hash(self.class_index)
        hash_value ^= hash(self.bbox)
        hash_value ^= hash(self.keypoints)
        return hash_value

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, YoloKeypointAnnotation):
            return NotImplemented
        if self.class_index != other.class_index:
            return False
        if self.bbox != other.bbox:
            return False
        if self.keypoints != other.keypoints:
            return False
        return True


@dataclass(eq=False)
class YoloKeypointImage:
    image_id: str = ""
    labels: set[YoloKeypointAnnotation] = field(default_factory=set)

    def to_txt(self) -> str:
        return "".join(label.to_row() for label in self.labels)

    @classmethod
    def from_txt(cls, data: str) -> YoloKeypointImage:
        self = cls()
        for line in data.splitlines():
            self.labels.add(YoloKeypointAnnotation.from_row(line))
        return self

    def __hash__(self) -> int:
        hash_value = hash(self.image_id)
        for label in self.labels:
            hash_value ^= hash(label)
        return hash_value

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, YoloKeypointImage):
            return NotImplemented
        if self.image_id != other.image_id:
            return False
        if self.labels != other.labels:
            return False
        return True


@dataclass
class YoloKeypointDataset:
    kpt_shape: tuple[int, int] = (2, 3)
    flip_idx: tuple[int, int] = (0, 1)
    names: tuple[ModelLabel, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict:
        data = to_dict(self)
        data["names"] = [label.value for label in self.names]
        data["nc"] = len(self.names)
        return data
