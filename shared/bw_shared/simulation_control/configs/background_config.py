from dataclasses import dataclass

from bw_shared.simulation_control.enums.simulation_scene import SimulationScene
from bw_shared.simulation_control.enums.simulation_skyimage import SimulationSkyImage


@dataclass
class BackgroundConfig:
    name: SimulationScene = SimulationScene.GARAGE_SCENE
    sky_image: SimulationSkyImage = SimulationSkyImage.EMPTY
