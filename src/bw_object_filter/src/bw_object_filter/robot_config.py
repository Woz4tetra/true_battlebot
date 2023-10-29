from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List


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
