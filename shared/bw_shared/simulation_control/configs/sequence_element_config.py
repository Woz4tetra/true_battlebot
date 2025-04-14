from dataclasses import dataclass

from bw_shared.simulation_control.enums.actor_role import ActorRole


@dataclass
class SequenceElementConfig:
    timestamp: float = 0.0

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    vroll: float = 0.0
    vpitch: float = 0.0
    vyaw: float = 0.0

    target_name: ActorRole = ActorRole.EMPTY
    secondary_target_name: ActorRole = ActorRole.EMPTY

    reset: bool = False
