import logging
from queue import Queue
from typing import Generic, Type, TypeVar

from perception_tools.rosbridge.ros_message_interface import RosMessageInterface
from perception_tools.rosbridge.types import RawRosMessage
from roslibpy import Message, Ros, Topic

T = TypeVar("T", bound=RosMessageInterface)


class RosPublisher(Generic[T]):
    topic: Topic
    log: bool = False
    log_filters: list[str] = []

    def __init__(
        self,
        ros: Ros,
        topic: str,
        msg_type: Type[T],
        queue_size: int = 1,
        latch: bool = False,
        compression: str | None = None,
    ):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: RawRosMessage | None = None
        else:
            self.queue: Queue[RawRosMessage] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.msg_type = msg_type
        self.topic_name = topic.replace("-", "_")
        self.topic = Topic(
            ros,
            self.topic_name,
            self.msg_type.type,
            queue_size=queue_size,
            latch=latch,
            compression=compression,
        )

    def publish(self, msg: T) -> None:
        if self.log and (len(self.log_filters) == 0 or self.topic_name in self.log_filters):
            self.logger.debug(f"{self.topic_name} published a message: {msg}")
        self.topic.publish(Message(msg.to_raw()))
