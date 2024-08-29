import logging
from queue import Empty, Full, Queue
from typing import Any, Generic, Type, TypeVar

import rospy

T = TypeVar("T")


class RosPollSubscriber(Generic[T]):
    log: bool = False
    exclude_filters: list[str] = []

    def __init__(self, topic: str, msg_type: Type[T], queue_size: int = 1, buff_size: int | None = None):
        self.queue_size = queue_size
        if queue_size == 1:
            self.last_value: T | None = None
        else:
            self.queue: Queue[T] = Queue(maxsize=queue_size)

        self.logger = logging.getLogger("perception")

        self.msg_type = msg_type
        self.topic_name = topic
        kwargs: dict[str, Any] = {"queue_size": queue_size}
        if buff_size is not None:
            kwargs["buff_size"] = buff_size
        self.subscriber = rospy.Subscriber(self.topic_name, msg_type, lambda msg: self._callback(msg), **kwargs)

    def receive(self) -> T | None:
        if self.queue_size != 1:
            return self._pop_message()
        return_val = self.last_value
        self.last_value = None
        return return_val

    def _callback(self, msg: T) -> None:
        if self.log and (len(self.exclude_filters) == 0 or self.topic_name not in self.exclude_filters):
            self.logger.debug(f"{self.topic_name} received a message")
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
