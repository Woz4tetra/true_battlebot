from dataclasses import dataclass
from typing import Optional


@dataclass
class PidConfig:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    kf: float = 0.0
    i_zone: Optional[float] = None
    i_max: float = 0.0
    tolerance: float = 0.0
