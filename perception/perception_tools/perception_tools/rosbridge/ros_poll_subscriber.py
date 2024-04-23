import logging
from queue import Empty, Full, Queue
from typing import Generic, Type, TypeVar

from perception_tools.rosbridge.ros_message_interface import RosMessageInterface
from perception_tools.rosbridge.types import RawRosMessage
from roslibpy import Ros, Topic

T = TypeVar("T", bound=RosMessageInterface)


class RosPollRawSubscriber(Generic[T]):
    topic: Topic

    def __init__(self, ros: Ros, topic: str, msg_type: str, queue_size: int):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: RawRosMessage | None = None
        else:
            self.queue: Queue[RawRosMessage] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.topic_name = topic.replace("-", "_")
        self.topic = Topic(ros, self.topic_name, msg_type, queue_size=queue_size)
        self.topic.subscribe(self._callback)

    def receive(self) -> RawRosMessage | None:
        return self._receive_raw()

    def _receive_raw(self) -> RawRosMessage | None:
        if self.queue_size != 1:
            return self._pop_message()
        return_val = self.last_value
        self.last_value = None
        return return_val

    def _callback(self, msg: RawRosMessage) -> None:
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

    def _pop_message(self) -> RawRosMessage | None:
        try:
            return_val = self.queue.get(block=False)
        except Empty:
            return_val = None
        return return_val


class RosPollSubscriber(RosPollRawSubscriber, Generic[T]):
    topic: Topic

    def __init__(self, ros: Ros, topic: str, msg_type: Type[T], queue_size: int):
        self.msg_type = msg_type
        super().__init__(ros, topic, msg_type.type, queue_size)

    def receive(self) -> T | None:
        raw_msg = self._receive_raw()
        if raw_msg is None:
            return None
        return self.msg_type.from_raw(raw_msg)
