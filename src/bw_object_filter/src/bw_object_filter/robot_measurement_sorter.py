import itertools
import math
from typing import Dict, List, Mapping

import numpy as np
from bw_tools.structs.pose2d import Pose2D
from geometry_msgs.msg import Pose

from bw_object_filter.filter_models.filter_model import FilterModel


class RobotMeasurementSorter:
    def __init__(self, filters: Mapping[int, FilterModel]) -> None:
        self.filters = filters
        self.filter_ids = [robot_id for robot_id in self.filters.keys()]
        self.cached_permutations = {}

    def get_distance(self, measurement: Pose, filter_model: FilterModel) -> float:
        pose = Pose2D.from_msg(measurement)
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
        num_measurements = distances.shape[0]
        num_filters = distances.shape[1]
        meas_indices = np.arange(num_measurements)
        if num_measurements not in self.cached_permutations:
            self.cached_permutations[num_measurements] = np.array(list(itertools.permutations(meas_indices)))
        permutations = self.cached_permutations[num_measurements]
        permutation_distances = np.sum(distances[permutations, tuple(np.arange(num_filters))], axis=1)
        min_index = np.argmin(permutation_distances)
        return permutations[min_index]

    def get_ids(self, measurements: List[Pose]) -> Dict[int, int]:
        measurement_ids = {}
        distances = np.zeros((len(measurements), len(self.filters)))
        for measurement_index, measurement in enumerate(measurements):
            measurement_ids[measurement_index] = measurement
            for filter_index, robot_id in enumerate(self.filter_ids):
                distances[measurement_index, filter_index] = self.get_distance(measurement, self.filters[robot_id])
        assigned_ids = {}
        minimized = self.minimize_permutation(distances)
        for filter_index, measurement_index in enumerate(minimized):
            robot_id = self.filter_ids[filter_index]
            assigned_ids[robot_id] = measurement_index
        return assigned_ids
