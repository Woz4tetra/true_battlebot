import logging
from queue import Empty, Full, Queue
from typing import Generic, Type, TypeVar

from perception_tools.rosbridge.ros_message_interface import RosMessageInterface
from perception_tools.rosbridge.types import RawRosMessage
from roslibpy import Ros, Topic

T = TypeVar("T", bound=RosMessageInterface)


class RosPollSubscriber(Generic[T]):
    topic: Topic
    log: bool = False
    log_filters: list[str] = []

    def __init__(self, ros: Ros, topic: str, msg_type: Type[T], queue_size: int = 1):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: T | None = None
        else:
            self.queue: Queue[T] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.msg_type = msg_type
        self.topic_name = topic
        self.topic = Topic(ros, self.topic_name, msg_type.type, queue_size=queue_size)
        self.topic.subscribe(self._callback)

    def receive(self) -> T | None:
        if self.queue_size != 1:
            return self._pop_message()
        return_val = self.last_value
        self.last_value = None
        return return_val

    def _callback(self, raw_msg: RawRosMessage) -> None:
        if self.log and (len(self.log_filters) == 0 or self.topic_name in self.log_filters):
            self.logger.debug(f"{self.topic_name} received a message: {raw_msg}")
        msg = self.msg_type.from_raw(raw_msg)
        if self.queue_size == 1:
            self.last_value = msg
            return

        while True:
            try:
                self.queue.put_nowait(msg)
                break
            except Full:
                self.logger.debug(f"{self.topic_name} dropping a message from the queue")
                self._pop_message()

    def _pop_message(self) -> T | None:
        try:
            return_val = self.queue.get(block=False)
        except Empty:
            return_val = None
        return return_val
