from typing import Dict, Mapping

from bw_interfaces.msg import EstimatedRobot, EstimatedRobotArray

from bw_object_filter.filter_models.filter_model import FilterModel


class RobotMeasurementSorter:
    def __init__(self, filters: Mapping[int, FilterModel]) -> None:
        self.filters = filters

    def get_ids(self, measurements: EstimatedRobotArray) -> Dict[int, EstimatedRobot]:
        pass
