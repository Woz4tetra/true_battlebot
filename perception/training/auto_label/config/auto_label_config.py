from __future__ import annotations

import logging
from dataclasses import dataclass, field

from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class TrackerConfig:
    sam2_model_config_path = "configs/sam2.1/sam2.1_hiera_l.yaml"
    checkpoint = "/home/bwbots/.cache/sam2/sam2.1_hiera_large.pt"
    interpolation_max_length: int = 500


@dataclass
class AutoLabelConfig:
    log_level: int = logging.DEBUG
    min_size: tuple[int, int] = (800, 600)
    max_size: tuple[int, int] = (3200, 2400)
    default_size: tuple[int, int] | None = None
    data_root_directory: str = ""
    default_jump_count: int = 300
    tracker: TrackerConfig = field(default_factory=TrackerConfig)

    @classmethod
    def from_dict(cls, config_dict: dict) -> AutoLabelConfig:
        return from_dict(cls, config_dict)

    def to_dict(self) -> dict:
        return to_dict(self)
