import time
from multiprocessing import Manager, Process
from typing import Any, Callable

from perception_tools.rosbridge.ros_publisher import RosPublisher, T


class RosThreadedPublisher(RosPublisher[T]):
    def __init__(
        self,
        topic: str,
        msg_type: type[T],
        publish_callback: Callable[[Any], T] | None = None,
        queue_size: int = 1,
        latch: bool = False,
    ):
        super().__init__(topic, msg_type, queue_size, latch)
        self.publish_callback = publish_callback
        self.manager = Manager()
        self.publish_queue = self.manager.Queue(maxsize=queue_size)
        self.publish_process = Process(target=self._publish_thread)
        self.publish_process.daemon = True
        self.publish_process.start()

    def _publish_thread(self) -> None:
        while True:
            data = self.publish_queue.get()
            if self.publish_callback is not None:
                msg = self.publish_callback(data)
            else:
                msg = data
            super().publish(msg)

    def publish(self, msg: Any) -> None:
        if self.publish_queue.full():
            return
        t0 = time.perf_counter()
        self.publish_queue.put(msg)
        t1 = time.perf_counter()
        print(f"publishing took {t1-t0} seconds")
