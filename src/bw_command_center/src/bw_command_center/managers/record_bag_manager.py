import os
import subprocess
import time
from threading import Thread
from typing import Callable

import rospy
from std_msgs.msg import String


class RecordBagManager:
    def __init__(
        self,
        base_record_dir: str,
        start_callback: Callable[[str], None],
        stop_callback: Callable[[str], None],
        timeout: float = 15.0,
        exclude_regex: str = "",
    ) -> None:
        self.start_callback = start_callback
        self.stop_callback = stop_callback
        self.timeout = timeout
        self.base_record_dir = base_record_dir
        self.base_record_node_name = "record_bag"
        self.record_node_name = ""
        self.recording_instance = 0
        self.exclude_regex = exclude_regex
        self.process = None
        self.path = ""
        self.bag_started_sub = rospy.Subscriber(
            "begin_write",
            String,
            self.bag_started_callback,
        )
        self.stop_thread = None
        self.is_stop_task_active = False

    def start(self, filename: str) -> None:
        if self.process is not None:
            rospy.logwarn("Bag recording already started")
            return
        self.path = os.path.join(self.base_record_dir, f"{filename}.bag")
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.record_node_name = f"{self.base_record_node_name}_{self.recording_instance}"
        self.recording_instance += 1
        self.process = subprocess.Popen(
            [
                "rosbag",
                "record",
                "-p",
                "-a",
                "-O",
                self.path,
                f"__name:={self.record_node_name}",
                "-x",
                self.exclude_regex,
            ]
        )

    def stop(self) -> None:
        rospy.logdebug(f"Record process is {'inactive' if self.process is None else 'active'}.")
        rospy.logdebug(f"Stop thread is {'inactive' if self.is_stop_task_active else 'active'}.")
        if self.process is None or self.is_stop_task_active:
            rospy.logwarn("Bag recording already stopped")
            return
        self.stop_thread = Thread(target=self._stop_thread_task)
        self.stop_thread.start()
        self.is_stop_task_active = True

    def _stop_thread_task(self) -> None:
        rospy.logdebug("Stop thread started")
        if self.process is None:
            return

        node = rospy.get_namespace() + self.record_node_name
        os.system(f"rosnode kill {node}")
        rospy.logdebug(f"Killed node {node}")

        self.wait_for_file(self.path, self.timeout)

        self.process.terminate()
        time.sleep(0.1)
        self.process.kill()
        rospy.logdebug(f"Terminated process {self.process.pid}")

        self.process = None
        self.bag_stopped_callback(self.path)
        self.is_stop_task_active = False

    def wait_for_file(self, path: str, timeout: float) -> None:
        rospy.logdebug(f"Waiting for {path} to be created")
        start = time.perf_counter()
        while not os.path.exists(path):
            if time.perf_counter() - start > timeout:
                raise TimeoutError(f"File {path} not created after {timeout} seconds")
            rospy.sleep(0.1)
        rospy.logdebug(f"File {path} created")

    def bag_started_callback(self, msg: String) -> None:
        rospy.loginfo(f"Bag recording started: {msg.data}")
        if not self.path.endswith(msg.data):
            rospy.logwarn(f"Bag recording started with unexpected name: {msg.data}")
        self.start_callback(self.path)

    def bag_stopped_callback(self, path: str) -> None:
        rospy.loginfo(f"Bag recording stopped: {path}")
        self.stop_callback(path)
