from dataclasses import dataclass
from typing import Literal


@dataclass
class YoloKeypointConfig:
    type: Literal["YoloKeypoint"] = "YoloKeypoint"
    model_name: str = "model.pt"
    threshold: float = 0.8
    debug: bool = False
