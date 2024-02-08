from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np
from dacite import from_dict


@dataclass
class LookupTable:
    frequencies: list[float] = field(default_factory=lambda: [])
    min_command: float = 0.0
    max_command: float = 0.0
    lower_cutoff: float = 0.0
    upper_cutoff: float = 0.0

    def __post_init__(self):
        self.velocities = np.arange(self.min_command, self.max_command + 1, 1)
        self.freq_array = np.array(self.frequencies, dtype=float)

    @classmethod
    def from_dict(cls, data: dict) -> LookupTable:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    def frequency_to_command(self, frequency: float) -> float:
        return float(self.velocities[np.argmin(np.abs(self.freq_array - frequency))])
