from dataclasses import dataclass

from bw_shared.simulation_control.enums.physics_material_combine_type import PhysicsMaterialCombineType


@dataclass
class PhysicsMaterialsConfig:
    name: str = "default"
    static_friction: float = 0.0
    dynamic_friction: float = 0.0
    bounciness: float = 0.0
    friction_combine: PhysicsMaterialCombineType = PhysicsMaterialCombineType.AVERAGE
    bounce_combine: PhysicsMaterialCombineType = PhysicsMaterialCombineType.AVERAGE
