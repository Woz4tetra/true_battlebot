from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from bw_shared.messages.dataclass_utils import from_dict, to_dict

EPSILON = 0.01


@dataclass
class LookupTableConfig:
    frequencies: list[float]
    velocities: list[float]
    lower_cutoff_frequency: float
    upper_cutoff_frequency: float

    def __post_init__(self):
        self.cutoff_frequency = min(abs(self.lower_cutoff_frequency), abs(self.upper_cutoff_frequency))
        self.cutoff_velocity = self.lookup_velocity(self.cutoff_frequency)

    def lookup_velocity(self, frequency: float) -> float:
        if abs(frequency) < EPSILON:
            return 0.0
        return float(np.interp(frequency, self.frequencies, self.velocities))

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_dict(cls, data: dict) -> LookupTableConfig:
        return from_dict(cls, data)
