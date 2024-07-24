from dataclasses import dataclass, field
from typing import Literal

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class YoloKeypointConfig:
    type: Literal["YoloKeypoint"] = "YoloKeypoint"
    model_path: str = "model.pt"
    metadata_path: str = "model_metadata.json"
    threshold: float = 0.85
    debug: bool = False
    model_to_system_labels: dict[ModelLabel, Label] = field(default_factory=dict)
