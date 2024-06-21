# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum

from bw_interfaces.msg import RobotConfigMsg, RobotFleetConfigMsg
from dacite import Config, from_dict

from bw_shared.enums.robot_team import RobotTeam


@dataclass
class RobotConfig:
    """
    This class is a dataclass that holds the configuration for a robot.
    """

    name: str
    team: RobotTeam = RobotTeam.OUR_TEAM
    ids: list[int] = field(default_factory=lambda: [])
    radius: float = 0.15  # max outer radius of the robot in meters

    @classmethod
    def from_dict(cls, data: dict) -> RobotConfig:
        return from_dict(data_class=cls, data=data, config=Config(cast=[Enum]))

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_msg(cls, msg: RobotConfigMsg) -> RobotConfig:
        return cls(name=msg.name, team=RobotTeam(msg.team), ids=msg.ids, radius=msg.radius)

    def to_msg(self) -> RobotConfigMsg:
        return RobotConfigMsg(name=self.name, team=self.team.value, ids=self.ids, radius=self.radius)


@dataclass
class RobotFleetConfig:
    robots: list[RobotConfig] = field(default_factory=lambda: [])

    @classmethod
    def from_dict(cls, data: dict) -> RobotFleetConfig:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_msg(cls, msg: RobotFleetConfigMsg) -> RobotFleetConfig:
        return cls(robots=[RobotConfig.from_msg(robot) for robot in msg.robots])

    def to_msg(self) -> RobotFleetConfigMsg:
        return RobotFleetConfigMsg(robots=[robot.to_msg() for robot in self.robots])
