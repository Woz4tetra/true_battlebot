from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import ModelLabel
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class LabelColor:
    r: float
    g: float
    b: float
    a: float = 1.0

    def to_cv_color(self) -> tuple[int, int, int]:
        return (int(self.b * 255), int(self.g * 255), int(self.r * 255))


LABEL_COLORS = {
    ModelLabel.BACKGROUND: LabelColor(0.0, 0.0, 0.0),
    ModelLabel.MR_STABS_MK1: LabelColor(1.0, 0.0, 0.327),
    ModelLabel.MRS_BUFF_MK1: LabelColor(0.0, 0.115, 1.0),
    ModelLabel.REFEREE: LabelColor(0.339, 0.339, 0.339),
    ModelLabel.ROBOT: LabelColor(1.0, 0.61, 0.0),
    ModelLabel.FIELD: LabelColor(0.0, 1.0, 0.0),
    ModelLabel.MR_STABS_MK2: LabelColor(0.69, 0.05, 0.05),
    ModelLabel.MRS_BUFF_MK2: LabelColor(0.0, 0.71, 0.92),
    ModelLabel.MAIN_BOT: LabelColor(1.0, 0.0, 0.327),
    ModelLabel.MINI_BOT: LabelColor(0.0, 0.115, 1.0),
    ModelLabel.OPPONENT_1: LabelColor(1.0, 0.61, 0.0),
    ModelLabel.OPPONENT_2: LabelColor(1.0, 0.71, 0.1),
}


@dataclass
class ModelMetadata:
    labels: list[ModelLabel]
    colors: list[LabelColor] = field(default_factory=list)
    keypoints: list[list[KeypointName]] = field(default_factory=list)

    def __post_init__(self):
        if not self.colors:
            self.colors = [LABEL_COLORS[label] for label in self.labels]
        if self.keypoints:
            assert len(self.labels) == len(self.keypoints)

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    @cached_property
    def color_map(self) -> dict[ModelLabel, LabelColor]:
        return dict(zip(self.labels, self.colors))

    @cached_property
    def keypoint_map(self) -> dict[ModelLabel, list[KeypointName]]:
        return dict(zip(self.labels, self.keypoints))


FIELD_SEMANTIC_MODEL_METADATA = ModelMetadata(
    labels=[ModelLabel.BACKGROUND, ModelLabel.FIELD],
)
