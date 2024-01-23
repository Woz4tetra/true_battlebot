from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum, auto
from typing import List

from dacite import from_dict


class RobotTeam(Enum):
    OUR_TEAM = auto()  # our team
    THEIR_TEAM = auto()  # their team
    REFEREE = auto()


@dataclass
class RobotConfig:
    """
    This class is a dataclass that holds the configuration for a red or blue team robot.
    """

    name: str
    up_id: int = -1
    down_id: int = -1
    radius: float = 0.0
    bridge_id: int = -1
    base_width: float = 1.0

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
    robots: List[RobotConfig] = field(default_factory=lambda: [])

    @classmethod
    def from_dict(cls, data: dict) -> RobotFleetConfig:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
