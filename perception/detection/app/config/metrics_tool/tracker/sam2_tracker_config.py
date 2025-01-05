from dataclasses import dataclass
from typing import Literal


@dataclass
class Sam2TrackerConfig:
    type: Literal["Sam2Tracker"] = "Sam2Tracker"

    checkpoint = "/home/bwbots/.cache/sam2/sam2.1_hiera_large.pt"
    model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
