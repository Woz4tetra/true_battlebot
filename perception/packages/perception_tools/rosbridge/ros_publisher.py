import logging
from typing import Generic, Type, TypeVar

import rospy

T = TypeVar("T")


class RosPublisher(Generic[T]):
    log: bool = False
    exclude_filters: list[str] = []

    def __init__(self, topic: str, msg_type: Type[T], queue_size: int = 1, latch: bool = False):
        self.queue_size = queue_size

        self.logger = logging.getLogger(self.__class__.__name__)

        self.msg_type = msg_type
        self.topic_name = topic.replace("-", "_")
        self.publisher = rospy.Publisher(
            self.topic_name,
            self.msg_type,
            queue_size=queue_size,
            latch=latch,
        )

    def publish(self, msg: T) -> None:
        if self.log and (len(self.exclude_filters) == 0 or self.topic_name not in self.exclude_filters):
            self.logger.debug(f"{self.topic_name} published a message")
        self.publisher.publish(msg)

    def num_subscribers(self) -> int:
        return int(self.publisher.get_num_connections())

    def unregister(self) -> None:
        self.publisher.unregister()
