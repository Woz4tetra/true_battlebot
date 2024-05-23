import itertools
import math
from typing import Dict, List, Mapping, Tuple

import numpy as np
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance

from bw_object_filter.filter_models.drive_kf_model import DriveKalmanModel


class RobotMeasurementSorter:
    def __init__(self, filters: Mapping[str, DriveKalmanModel]) -> None:
        self.filters = filters
        self.filter_names = [robot_name for robot_name in self.filters.keys()]
        self.cached_permutations: dict[tuple[int, int], np.ndarray] = {}

    def get_distance(self, measurement: PoseWithCovariance, filter_model: DriveKalmanModel) -> float:
        pose = Pose2D.from_msg(measurement.pose)
        filter_pose = Pose2D.from_msg(filter_model.get_state()[0].pose)
        dx = pose.x - filter_pose.x
        dy = pose.y - filter_pose.y
        return math.sqrt(dx * dx + dy * dy)

    def minimize_permutation(self, distances: np.ndarray) -> List[Tuple[int, int]]:
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
        should_flip = num_measurements < num_filters
        if should_flip:
            num_measurements, num_filters = num_filters, num_measurements
            distances = distances.T
        indices = np.arange(num_measurements)
        key = (num_measurements, num_filters)
        if key not in self.cached_permutations:
            self.cached_permutations[key] = np.array(list(itertools.permutations(indices, num_filters)))
        permutations = self.cached_permutations[key]
        permutation_distances = np.sum(distances[permutations, tuple(np.arange(num_filters))], axis=1)
        min_index = np.argmin(permutation_distances)
        mapping = []
        for matched_column, matched_row in enumerate(permutations[min_index]):
            if should_flip:
                mapping.append((matched_row, matched_column))
            else:
                mapping.append((matched_column, matched_row))
        return mapping

    def get_ids(self, measurements: List[PoseWithCovariance]) -> Dict[str, int]:
        if len(measurements) == 0 or len(self.filters) == 0:
            return {}
        measurement_ids = {}
        distances = np.zeros((len(measurements), len(self.filters)))
        for measurement_index, measurement in enumerate(measurements):
            measurement_ids[measurement_index] = measurement
            for filter_index, robot_name in enumerate(self.filter_names):
                distances[measurement_index, filter_index] = self.get_distance(measurement, self.filters[robot_name])
        assigned_ids = {}
        minimized = self.minimize_permutation(distances)
        for filter_index, measurement_index in minimized:
            robot_name = self.filter_names[filter_index]
            assigned_ids[robot_name] = measurement_index
        return assigned_ids
