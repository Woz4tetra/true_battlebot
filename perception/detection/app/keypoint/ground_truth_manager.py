from nav_msgs.msg import Odometry
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import CameraInfo


class GroundTruthManager:
    def __init__(
        self, ground_truth_subs: list[RosPollSubscriber[Odometry]], camera_info_sub: RosPollSubscriber[CameraInfo]
    ) -> None:
        self.ground_truth_subs = ground_truth_subs
        self.camera_info_sub = camera_info_sub
        self.robots: dict[str, Odometry] = {}
        self.camera_info: CameraInfo | None = None

    def get_camera_info(self) -> CameraInfo | None:
        if self.camera_info is None:
            self.camera_info = self.camera_info_sub.receive()
        return self.camera_info

    def get_robots(self) -> list[Odometry]:
        for sub in self.ground_truth_subs:
            if odom := sub.receive():
                self.robots[odom.child_frame_id] = odom
        return list(self.robots.values())
