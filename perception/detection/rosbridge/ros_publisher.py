import logging
from queue import Queue

from rosbridge.types import RawRosMessage
from roslibpy import Message, Ros, Topic


class RosPublisher:
    topic: Topic

    def __init__(self, ros: Ros, topic: str, msg_type: str, queue_size: int, latch: bool = False):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: RawRosMessage | None = None
        else:
            self.queue: Queue[RawRosMessage] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.topic_name = topic.replace("-", "_")
        self.topic = Topic(ros, self.topic_name, msg_type, queue_size=queue_size, latch=latch)

    def publish(self, msg: RawRosMessage) -> None:
        self.topic.publish(Message(msg))