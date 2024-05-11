from abc import abstractmethod
from typing import Tuple

from bw_tools.structs.xy import XY
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance

from .helpers import StateSquareMatrix


class FilterModel:
    @abstractmethod
    def predict(self) -> None: ...

    @abstractmethod
    def update_position(self, msg: PoseWithCovarianceStamped) -> None: ...

    @abstractmethod
    def update_pose(self, msg: PoseWithCovarianceStamped) -> None: ...

    @abstractmethod
    def update_cmd_vel(self, msg: TwistWithCovariance) -> None: ...

    @abstractmethod
    def get_state(self) -> Tuple[PoseWithCovariance, TwistWithCovariance]: ...

    @abstractmethod
    def get_covariance(self) -> StateSquareMatrix: ...

    @abstractmethod
    def teleport(self, msg: PoseWithCovarianceStamped) -> None: ...

    @abstractmethod
    def clamp_bounds(self, lower_bound: XY, upper_bound: XY) -> None: ...
