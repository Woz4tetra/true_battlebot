from dataclasses import dataclass
from typing import Literal


@dataclass
class NoopSegmentationConfig:
    type: Literal["NoopSegmentation"] = "NoopSegmentation"
