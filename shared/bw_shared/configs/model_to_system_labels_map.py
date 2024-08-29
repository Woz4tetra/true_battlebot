from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

from bw_interfaces.msg import LabelMap

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class ModelToSystemLabelsMap:
    mapping: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        self.labels = {ModelLabel(label): Label(system_label) for label, system_label in self.mapping.items()}

    def get_class_indices(self, model_labels: Iterable[ModelLabel]) -> dict[Label, int]:
        class_indices = {}
        for idx, model_label in enumerate(model_labels):
            if model_label in self.labels:
                class_indices[self.labels[model_label]] = idx
        return class_indices

    def to_msg(self) -> LabelMap:
        model_labels = [model_label.value for model_label in self.labels.keys()]
        system_labels = [self.labels[model_label].value for model_label in self.labels.keys()]
        return LabelMap(model_labels=model_labels, system_labels=system_labels)

    @classmethod
    def from_msg(cls, msg: LabelMap) -> ModelToSystemLabelsMap:
        mapping = {model_label: system_label for model_label, system_label in zip(msg.model_labels, msg.system_labels)}
        return cls(mapping=mapping)
