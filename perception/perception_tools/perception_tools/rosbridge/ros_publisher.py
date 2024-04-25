import logging
from queue import Queue
from typing import Generic, Type, TypeVar

from perception_tools.rosbridge.ros_message_interface import RosMessageInterface
from perception_tools.rosbridge.types import RawRosMessage
from roslibpy import Message, Ros, Topic

T = TypeVar("T", bound=RosMessageInterface)


class RosPublisher(Generic[T]):
    topic: Topic

    def __init__(self, ros: Ros, topic: str, msg_type: Type[T], queue_size: int, latch: bool = False):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: RawRosMessage | None = None
        else:
            self.queue: Queue[RawRosMessage] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.msg_type = msg_type
        self.topic_name = topic.replace("-", "_")
        self.topic = Topic(ros, self.topic_name, self.msg_type.type, queue_size=queue_size, latch=latch)

    def publish(self, msg: T) -> None:
        self.topic.publish(Message(msg.to_raw()))
