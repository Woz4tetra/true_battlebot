from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber


class GroundTruthManager:
    def __init__(self, robots_sub: RosPollSubscriber[EstimatedObjectArray]) -> None:
        self.robots_sub = robots_sub
        self.robots: dict[str, EstimatedObject] = {}

    def get_robots(self) -> list[EstimatedObject]:
        if robots := self.robots_sub.receive():
            for robot in robots.robots:
                self.robots[robot.label] = robot
        return list(self.robots.values())
