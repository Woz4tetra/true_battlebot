#!/usr/bin/env python
import datetime
from threading import Event

import rospy
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import SystemSummary
from bw_shared.environment import get_robot
from bw_tools.get_param import get_param
from bw_tools.structs.cage_corner import CageCorner
from bw_tools.system_info import get_system_info
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from bw_command_center.managers.record_bag_manager import RecordBagManager
from bw_command_center.managers.svo_service_manager import SvoServiceManager


class WebappRelay:
    def __init__(self) -> None:
        cameras = get_param("~cameras", ["camera_0"])
        exclude_regex = get_param("~exclude_regex", "")

        self.system_summary_pub = rospy.Publisher("system_summary", SystemSummary, queue_size=1, latch=True)
        self.svo_service_managers = [SvoServiceManager(camera, "/media/storage/svo") for camera in cameras]
        self.bag_manager = RecordBagManager(
            "/media/storage/bags", self.start_callback, self.stop_callback, exclude_regex=exclude_regex
        )
        self.set_record_srv = rospy.Service("set_record", SetBool, self.set_record_callback)
        self.is_recording = False
        self.callback_event = Event()

        self.cage_corner_pub = rospy.Publisher("set_cage_corner", RosCageCorner, queue_size=1, latch=True)
        self.cage_corner_pub.publish(CageCorner.DOOR_SIDE.to_msg())

    def publish_system_summary(self) -> None:
        self.system_summary_pub.publish(get_system_info())

    def set_record_callback(self, req: SetBoolRequest) -> SetBoolResponse:
        if req.data and not self.is_recording:
            rospy.loginfo("Start recording")
            now = datetime.datetime.now()
            datestr = now.strftime("%Y-%m-%dT%H-%M-%S")
            name = f"{get_robot()}_{datestr}"
            for manager in self.svo_service_managers:
                manager.start(name)
            self.bag_manager.start(name)
            self.await_recording_state(True)
            return SetBoolResponse(success=True)
        elif not req.data and self.is_recording:
            rospy.loginfo("Stop recording")
            for manager in self.svo_service_managers:
                manager.stop()
            self.bag_manager.stop()
            self.await_recording_state(False)
            return SetBoolResponse(success=True)
        else:
            rospy.loginfo(f"Invalid request: {req.data}. Is recording: {self.is_recording}")
            return SetBoolResponse(success=False)

    def await_recording_state(self, is_recording: bool) -> None:
        rospy.logdebug(f"Awaiting recording state: {is_recording}")
        self.callback_event.wait()
        self.callback_event.clear()
        self.is_recording = is_recording

    def run(self) -> None:
        self.publish_system_summary()
        rospy.spin()

    def start_callback(self, path: str) -> None:
        self.callback_event.set()
        rospy.logdebug("Start callback")

    def stop_callback(self, path: str) -> None:
        self.callback_event.set()
        rospy.logdebug("Stop callback")


def main():
    rospy.init_node("webapp_connector", log_level=rospy.DEBUG)
    WebappRelay().run()


if __name__ == "__main__":
    main()
