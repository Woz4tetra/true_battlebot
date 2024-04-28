# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import asdict, dataclass, field

from dacite import from_dict

from bw_shared.enums.robot_team import RobotTeam


@dataclass
class RobotConfig:
    """
    This class is a dataclass that holds the configuration for a red or blue team robot.
    """

    name: str
    up_id: int = -1
    down_id: int = -1
    radius: float = 0.0  # outer radius of the robot in meters
    bridge_id: int = -1
    base_width: float = 1.0  # distance between the wheels

    @property
    def team(self) -> RobotTeam:
        if "referee" in self.name:
            return RobotTeam.REFEREE
        else:
            return RobotTeam.OUR_TEAM if self.up_id >= 0 or self.down_id >= 0 else RobotTeam.THEIR_TEAM

    @classmethod
    def from_dict(cls, data: dict) -> RobotConfig:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class RobotFleetConfig:
    robots: list[RobotConfig] = field(default_factory=lambda: [])

    @classmethod
    def from_dict(cls, data: dict) -> RobotFleetConfig:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
