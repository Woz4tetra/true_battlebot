import itertools
import math
from typing import Dict, List, Mapping

import numpy as np
from bw_interfaces.msg import EstimatedRobot, EstimatedRobotArray
from bw_tools.structs.pose2d import Pose2D

from bw_object_filter.filter_models.filter_model import FilterModel


class RobotMeasurementSorter:
    def __init__(self, filters: Mapping[int, FilterModel]) -> None:
        self.filters = filters
        self.filter_ids = [robot_id for robot_id in self.filters.keys()]

    def get_distance(self, measurement: EstimatedRobot, filter_model: FilterModel) -> float:
        pose = Pose2D.from_msg(measurement.pose)
        filter_pose = Pose2D.from_msg(filter_model.get_state()[0].pose)
        dx = pose.x - filter_pose.x
        dy = pose.y - filter_pose.y
        return math.sqrt(dx * dx + dy * dy)

    def minimize_permutation(self, distances: np.ndarray) -> np.ndarray:
        """
        this function minimizes permutations of distances
        example:
            distances   filter id
                            0    1    2
        measurements id
                        0   1    4    10
                        1   3.5  2    5
                        2   7    1.5  4
                        3   7    4    1.9

        optimal result:
        [0, 2, 3]

        Sums to 4.9

        The result has one element for each filter encoding which measurement index to assign to the filter
        """
        n = distances.shape[0]
        m = distances.shape[1]
        indices = np.arange(n)
        permutations = np.array(list(itertools.permutations(indices, m)))
        permutation_distances = np.sum(distances[permutations, tuple(np.arange(m))], axis=1)
        min_index = np.argmin(permutation_distances)
        return permutations[min_index]

    def get_ids(self, measurements: EstimatedRobotArray) -> Dict[int, EstimatedRobot]:
        measurement_ids = {}
        measurement_array: List[EstimatedRobot] = measurements.robots  # type: ignore
        distances = np.zeros((len(measurement_array), len(self.filters)))
        for measurement_index, measurement in enumerate(measurement_array):
            measurement: EstimatedRobot
            measurement_ids[measurement_index] = measurement
            for filter_index, robot_id in enumerate(self.filter_ids):
                distances[measurement_index, filter_index] = self.get_distance(measurement, self.filters[robot_id])
        assigned_ids = {}
        minimized = self.minimize_permutation(distances)
        for filter_index, measurement_index in enumerate(minimized):
            robot_id = self.filter_ids[filter_index]
            measurement = measurement_ids[measurement_index]
            assigned_ids[robot_id] = measurement
        return assigned_ids
