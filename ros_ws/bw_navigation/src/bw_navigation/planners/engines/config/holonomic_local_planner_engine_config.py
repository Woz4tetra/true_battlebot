from dataclasses import dataclass, field

from bw_shared.pid.config import PidConfig


@dataclass
class HolonomicLocalPlannerEngineConfig:
    linear_x_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
    linear_y_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
    angular_pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
