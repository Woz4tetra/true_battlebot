import numpy as np
from bw_shared.configs.robot_fleet_config import RobotConfig
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

from .helpers import pose_to_measurement
from .model_base import ModelBase


class TrackingModel(ModelBase):
    def __init__(
        self,
        config: RobotConfig,
        dt: float,
        stale_timeout: float = 10.0,
        robot_min_radius: float = 0.05,
        robot_max_radius: float = 0.15,
    ) -> None:
        super().__init__(config, stale_timeout, robot_min_radius, robot_max_radius)
        self.dt = dt

    def predict(self) -> None:
        with self.lock:
            self._clamp_divergent()

    def update_pose(self, msg: PoseWithCovariance) -> None:
        if not self._is_initialized:
            self.teleport(msg)
        with self.lock:
            measurement, noise = pose_to_measurement(msg)
            measurement = np.nan_to_num(measurement, copy=False)
            self.state = measurement
            self.covariance = noise
            self.reset_stale_timer()

    def update_position(self, msg: PoseWithCovariance) -> None:
        if not self._is_initialized:
            return
        # TODO: Implement this method

    def update_orientation(self, yaw: float, covariance: np.ndarray) -> None:
        # TODO: Implement this method
        pass

    def update_cmd_vel(self, msg: TwistWithCovariance) -> None:
        # TODO: Implement this method
        pass
