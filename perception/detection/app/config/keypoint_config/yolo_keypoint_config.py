from dataclasses import dataclass, field
from typing import Literal


@dataclass
class YoloKeypointConfig:
    type: Literal["YoloKeypoint"] = "YoloKeypoint"
    model_name: str = "model.pt"
    threshold: float = 0.8
    debug: bool = False
    model_to_system_labels: dict[str, str] = field(default_factory=dict)
