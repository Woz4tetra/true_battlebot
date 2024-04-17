import os

import rospy
from zed_interfaces.srv import start_svo_recording, stop_svo_recording


class SvoServiceManager:
    def __init__(self, service_prefix: str, base_record_dir: str) -> None:
        self.service_prefix = service_prefix
        self.base_record_dir = base_record_dir
        self.start_svo = rospy.ServiceProxy(os.path.join(service_prefix, "start_svo_recording"), start_svo_recording)
        self.stop_svo = rospy.ServiceProxy(os.path.join(service_prefix, "stop_svo_recording"), stop_svo_recording)
        self.is_recording = False

    def start(self, filename: str) -> None:
        if self.is_recording:
            rospy.logwarn(f"{self.service_prefix} SVO recording already started")
            return
        os.makedirs(self.base_record_dir, exist_ok=True)
        path = os.path.join(self.base_record_dir, f"{filename}.svo")
        self.is_recording = True
        try:
            self.start_svo(path)
        except rospy.ServiceException as e:
            rospy.logerr(f"{self.service_prefix} SVO recording failed to start: {e}")
            self.is_recording = False

    def stop(self) -> None:
        if not self.is_recording:
            rospy.logwarn(f"{self.service_prefix} SVO recording already stopped")
            return
        self.is_recording = False
        try:
            self.stop_svo()
        except rospy.ServiceException as e:
            rospy.logerr(f"{self.service_prefix} SVO recording failed to stop: {e}")
