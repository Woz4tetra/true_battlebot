from dataclasses import dataclass, field

from bw_shared.pid.config import PidConfig


@dataclass
class NearGoalConfig:
    linear_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
    angular_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=5.0, ki=0.0, kd=0.0, kf=1.0))
