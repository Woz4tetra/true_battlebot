from __future__ import annotations

from dataclasses import dataclass, field

from bw_shared.simulation_control.compute_camera_pose import compute_camera_pose
from bw_shared.simulation_control.configs.scenario_init_config import ScenarioInitConfig


@dataclass
class CameraConfig:
    init: ScenarioInitConfig = field(default_factory=ScenarioInitConfig)

    @classmethod
    def from_distance_rotation(cls, distance: float, elevation_angle: float, azimuth_angle: float) -> CameraConfig:
        pose = compute_camera_pose(distance, azimuth_angle, elevation_angle)
        return cls(init=ScenarioInitConfig.from_transform(pose))
