from __future__ import annotations

from dataclasses import dataclass

import toml
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import ModelLabel
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class KeypointsConfig:
    labels: list[ModelLabel]
    keypoint_names: dict[str, list[str]]

    def __post_init__(self) -> None:
        for label in self.labels:
            if label not in self.keypoint_names:
                raise ValueError(f"Keypoint names not provided for label {label}")
        for label in self.keypoint_names:
            if label not in self.labels:
                raise ValueError(f"Keypoint names provided for label {label} which is not in labels")
        self.keypoint_mapping = {
            ModelLabel(label): [KeypointName(name) for name in names] for label, names in self.keypoint_names.items()
        }

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_dict(cls, data: dict) -> KeypointsConfig:
        return from_dict(cls, data)


def load_keypoints_config(config_path: str) -> KeypointsConfig:
    with open(config_path, "r") as f:
        config = toml.load(f)
    return KeypointsConfig.from_dict(config)
