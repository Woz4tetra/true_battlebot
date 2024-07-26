from dataclasses import dataclass, field

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class ModelToSystemLabelsConfig:
    mapping: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        self.labels = {ModelLabel(label): Label(system_label) for label, system_label in self.mapping.items()}
