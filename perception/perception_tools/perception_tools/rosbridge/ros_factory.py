from typing import Type, TypeVar

from perception_tools.rosbridge.ros_message_interface import RosMessageInterface
from perception_tools.rosbridge.ros_poll_subscriber import RosPollRawSubscriber, RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from roslibpy import Ros

T = TypeVar("T", bound=RosMessageInterface)


class RosFactory:
    def __init__(self, host: str, port: int) -> None:
        self.ros = Ros(host, port)

    def connect(self) -> None:
        self.ros.run()

    def disconnect(self) -> None:
        self.ros.close()

    def make_subscriber(self, topic: str, msg_type: Type[T], queue_size: int) -> RosPollSubscriber[T]:
        return RosPollSubscriber(self.ros, topic, msg_type, queue_size)

    def make_raw_subscriber(self, topic: str, msg_type: str, queue_size: int) -> RosPollRawSubscriber:
        return RosPollRawSubscriber(self.ros, topic, msg_type, queue_size)

    def make_publisher(self, topic: str, msg_type: Type[T], queue_size: int, latch: bool = False) -> RosPublisher[T]:
        return RosPublisher(self.ros, topic, msg_type, queue_size, latch)
