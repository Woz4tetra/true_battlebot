from __future__ import annotations

from dataclasses import dataclass

from bw_shared.geometry.transform3d import Transform3D
from bw_shared.simulation_control.enums.scenario_init_type import ScenarioInitType


@dataclass
class ScenarioInitConfig:
    type: ScenarioInitType = ScenarioInitType.ABSOLUTE
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    z: float = 0.0  # meters
    roll: float = 0.0  # degrees
    pitch: float = 0.0  # degrees
    yaw: float = 0.0  # degrees

    @classmethod
    def from_transform(
        cls, transform: Transform3D, type: ScenarioInitType = ScenarioInitType.ABSOLUTE
    ) -> ScenarioInitConfig:
        angles = transform.rpy.to_degrees()
        return cls(
            type=type,
            x=transform.x,
            y=transform.y,
            z=transform.z,
            roll=angles[0],
            pitch=angles[1],
            yaw=angles[2],
        )
