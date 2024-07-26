from dataclasses import dataclass, field
from typing import Dict

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class ModelToSystemLabelsConfig:
    mapping: Dict[ModelLabel, Label] = field(default_factory=dict)
