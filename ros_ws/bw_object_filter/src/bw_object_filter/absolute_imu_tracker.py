class AbsoluteImuTracker:
    def __init__(self) -> None:
        self.absolute_reference = 0.0
        self.imu_reference = 0.0
        self.last_imu = 0.0

    def set_references(self, absolute_reference: float) -> None:
        self.absolute_reference = absolute_reference
        self.imu_reference = self.last_imu

    def get_absolute_yaw(self, imu_yaw: float) -> float:
        self.last_imu = imu_yaw
        return self.absolute_reference + (imu_yaw - self.imu_reference)
