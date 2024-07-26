from dataclasses import dataclass, field
from typing import Dict

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class ModelToSystemLabelsConfig:
    mapping: Dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        self.mapping = {ModelLabel(label): Label(system_label) for label, system_label in self.mapping.items()}
