from rosbridge.ros_poll_subscriber import RosPollSubscriber
from rosbridge.ros_publisher import RosPublisher
from roslibpy import Ros


class RosFactory:
    def __init__(self, host: str, port: int) -> None:
        self.ros = Ros(host, port)

    def connect(self) -> None:
        self.ros.run()

    def disconnect(self) -> None:
        self.ros.close()

    def make_subscriber(self, topic: str, msg_type: str, queue_size: int) -> RosPollSubscriber:
        return RosPollSubscriber(self.ros, topic, msg_type, queue_size)

    def make_publisher(self, topic: str, msg_type: str, queue_size: int, latch: bool = False) -> RosPublisher:
        return RosPublisher(self.ros, topic, msg_type, queue_size, latch)
