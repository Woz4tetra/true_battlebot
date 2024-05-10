from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

from .helpers import StateSquareMatrix


class FilterModel:
    @abstractmethod
    def predict(self) -> None:
        pass

    @abstractmethod
    def update_position(self, msg: PoseWithCovarianceStamped) -> None:
        pass

    @abstractmethod
    def update_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pass

    @abstractmethod
    def get_state(self) -> PoseWithCovariance:
        pass

    @abstractmethod
    def get_covariance(self) -> StateSquareMatrix:
        pass

    @abstractmethod
    def teleport(self, msg: PoseWithCovarianceStamped) -> None:
        pass
