from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List

from bw_tools.dataclass_deserialize import dataclass_deserialize


class RobotTeam(Enum):
    RED = auto()  # our team
    BLUE = auto()  # their team


OUR_TEAM = RobotTeam.RED
THEIR_TEAM = RobotTeam.BLUE


@dataclass
class RobotConfig:
    """
    This class is a dataclass that holds the configuration for a red or blue team robot.
    """

    name: str
    id: int

    @property
    def team(self) -> RobotTeam:
        return OUR_TEAM if self.id >= 0 else THEIR_TEAM


@dataclass
class RobotFleetConfig:
    robots: List[RobotConfig] = field(default_factory=lambda: [])

    def __post_init__(self) -> None:
        ids = [bot.id for bot in self.robots]
        if len(ids) != len(set(ids)):
            raise ValueError("Robot ids must be unique")
        names = [bot.name for bot in self.robots]
        if len(names) != len(set(names)):
            raise ValueError("Robot names must be unique")

    @classmethod
    def from_config(cls, config: Dict) -> RobotFleetConfig:
        return dataclass_deserialize(RobotFleetConfig, config)  # type: ignore
