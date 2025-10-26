from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Literal

from bw_shared.messages.dataclass_utils import from_dict, to_dict

InterpolatorType = Literal["cotracker_sam2"]


@dataclass
class TrackerConfig:
    sam2_model_config_path: str = "configs/sam2.1/sam2.1_hiera_l.yaml"
    sam2_checkpoint: str = "/home/bwbots/.cache/sam2/sam2.1_hiera_large.pt"
    cotracker_checkpoint: str = "/home/bwbots/.cache/co-tracker/scaled_offline.pth"
    interpolation_max_length: int = 20
    sam2_batch_size: int = 10
    interpolator_type: InterpolatorType = "cotracker_sam2"


@dataclass
class AutoLabelConfig:
    log_level: int = logging.DEBUG
    min_size: tuple[int, int] = (800, 600)
    max_size: tuple[int, int] = (3200, 2400)
    default_size: tuple[int, int] | None = None
    data_root_directory: str = ""
    default_jump_count: int = 1
    tracker: TrackerConfig = field(default_factory=TrackerConfig)

    @classmethod
    def from_dict(cls, config_dict: dict) -> AutoLabelConfig:
        return from_dict(cls, config_dict)

    def to_dict(self) -> dict:
        return to_dict(self)
