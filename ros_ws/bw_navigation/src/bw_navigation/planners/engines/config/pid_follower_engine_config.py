from dataclasses import dataclass
from typing import Optional

from bw_shared.pid.config import PidConfig


@dataclass
class PidFollowerEngineConfig:
    linear_pid: PidConfig = PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0)
    angular_pid: PidConfig = PidConfig(kp=6.0, ki=0.0, kd=0.0, kf=1.0)
    always_face_forward: bool = False
    clamp_linear: Optional[tuple[float, float]] = None
    clamp_angular: Optional[tuple[float, float]] = None
