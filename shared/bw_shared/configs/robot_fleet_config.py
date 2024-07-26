# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import dataclass, field

from bw_interfaces.msg import RobotConfigMsg, RobotFleetConfigMsg

from bw_shared.configs.tag_config import TagConfig
from bw_shared.enums.robot_team import RobotTeam
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class RobotConfig:
    """
    This class is a dataclass that holds the configuration for a robot.
    """

    name: str
    team: RobotTeam = RobotTeam.OUR_TEAM
    tags: list[TagConfig] = field(default_factory=list)
    radius: float = 0.15  # max outer radius of the robot in meters
    is_controlled: bool = False

    @classmethod
    def from_dict(cls, data: dict) -> RobotConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_msg(cls, msg: RobotConfigMsg) -> RobotConfig:
        return cls(
            name=msg.name,
            team=RobotTeam(msg.team),
            tags=[TagConfig.from_msg(tag) for tag in msg.tags],
            radius=msg.radius,
            is_controlled=msg.is_controlled,
        )

    def to_msg(self) -> RobotConfigMsg:
        return RobotConfigMsg(
            name=self.name,
            team=self.team.value,
            tags=[tag.to_msg() for tag in self.tags],
            radius=self.radius,
            is_controlled=self.is_controlled,
        )


@dataclass
class RobotFleetConfig:
    robots: list[RobotConfig] = field(default_factory=lambda: [])

    @classmethod
    def from_dict(cls, data: dict) -> RobotFleetConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_msg(cls, msg: RobotFleetConfigMsg) -> RobotFleetConfig:
        return cls(robots=[RobotConfig.from_msg(robot) for robot in msg.robots])

    def to_msg(self) -> RobotFleetConfigMsg:
        return RobotFleetConfigMsg(robots=[robot.to_msg() for robot in self.robots])
