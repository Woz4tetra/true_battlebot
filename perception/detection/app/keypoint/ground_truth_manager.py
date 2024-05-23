from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import CameraInfo


class GroundTruthManager:
    def __init__(
        self, robots_sub: RosPollSubscriber[EstimatedObjectArray], camera_info_sub: RosPollSubscriber[CameraInfo]
    ) -> None:
        self.robots_sub = robots_sub
        self.camera_info_sub = camera_info_sub
        self.robots: dict[str, EstimatedObject] = {}
        self.camera_info: CameraInfo | None = None

    def get_camera_info(self) -> CameraInfo | None:
        if self.camera_info is None:
            self.camera_info = self.camera_info_sub.receive()
        return self.camera_info

    def get_robots(self) -> list[EstimatedObject]:
        if robots := self.robots_sub.receive():
            for robot in robots.robots:
                self.robots[robot.label] = robot
        return list(self.robots.values())
