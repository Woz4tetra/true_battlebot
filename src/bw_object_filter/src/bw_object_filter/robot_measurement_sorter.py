import math
from typing import Dict, Mapping

from bw_interfaces.msg import EstimatedRobot, EstimatedRobotArray
from bw_tools.structs.pose2d import Pose2D

from bw_object_filter.filter_models.filter_model import FilterModel


class RobotMeasurementSorter:
    def __init__(self, filters: Mapping[int, FilterModel]) -> None:
        self.filters = filters

    def get_distance(self, measurement: EstimatedRobot, filter_model: FilterModel) -> float:
        pose = Pose2D.from_msg(measurement.pose)
        filter_pose = Pose2D.from_msg(filter_model.get_state()[0].pose)
        dx = pose.x - filter_pose.x
        dy = pose.y - filter_pose.y
        return math.sqrt(dx * dx + dy * dy)

    def get_ids(self, measurements: EstimatedRobotArray) -> Dict[int, EstimatedRobot]:
        pass
