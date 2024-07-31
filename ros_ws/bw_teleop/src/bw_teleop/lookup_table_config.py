from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from bw_shared.messages.dataclass_utils import from_dict, to_dict

EPSILON = 0.01


@dataclass
class LookupTableConfig:
    frequencies: list[float]
    min_command: float
    max_command: float
    lower_cutoff_frequency: float
    upper_cutoff_frequency: float

    def __post_init__(self):
        self.velocities = np.linspace(self.min_command, self.max_command, len(self.frequencies), endpoint=True)

    def lookup_velocity(self, frequency: float) -> float:
        if abs(frequency) < EPSILON:
            return 0.0
        elif self.lower_cutoff_frequency <= frequency <= self.upper_cutoff_frequency:
            frequency = self.lower_cutoff_frequency if frequency < 0.0 else self.upper_cutoff_frequency
        return float(np.interp(frequency, self.frequencies, self.velocities))

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_dict(cls, data: dict) -> LookupTableConfig:
        return from_dict(cls, data)
