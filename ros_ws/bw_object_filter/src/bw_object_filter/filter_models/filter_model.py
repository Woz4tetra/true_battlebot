from abc import abstractmethod
from typing import Tuple

import numpy as np
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance


class FilterModel:
    @abstractmethod
    def predict(self) -> None:
        pass

    @abstractmethod
    def update_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pass

    @abstractmethod
    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        pass

    @abstractmethod
    def get_state(self) -> Tuple[PoseWithCovariance, TwistWithCovariance]:
        pass

    @abstractmethod
    def get_covariance(self) -> np.ndarray:
        pass

    @abstractmethod
    def teleport(self, msg: PoseWithCovarianceStamped) -> None:
        pass
