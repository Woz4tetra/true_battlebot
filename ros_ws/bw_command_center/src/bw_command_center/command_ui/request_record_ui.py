import datetime
import tkinter as tk
from enum import Enum

import rospy
from bw_tools.environment import get_robot
from bw_tools.typing import get_param

from bw_command_center.command_ui.ui_base import UiBase
from bw_command_center.managers.record_bag_manager import RecordBagManager
from bw_command_center.managers.svo_service_manager import SvoServiceManager


class RecordButtonState:
    class State(Enum):
        START = "Start"
        STOP = "Stop"

    def __init__(self) -> None:
        self.state = RecordButtonState.State.START

    def get_text(self) -> str:
        return self.state.value

    def set_start(self) -> None:
        self.state = RecordButtonState.State.START

    def set_stop(self) -> None:
        self.state = RecordButtonState.State.STOP

    def get_state(self) -> State:
        return self.state


class RequestRecordUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window

        cameras = get_param("~cameras", ["camera_0"])
        exclude_regex = get_param("~exclude_regex", "")

        self.button_text = tk.StringVar()
        self.button_state = RecordButtonState()
        self.svo_service_managers = [SvoServiceManager(camera, "/media/storage/svo") for camera in cameras]
        self.bag_manager = RecordBagManager(
            "/media/storage/bags", self.start_callback, self.stop_callback, exclude_regex=exclude_regex
        )

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        tk.Label(master=left_frame, text="Recording:").pack(anchor=tk.W)
        tk.Button(
            master=right_frame,
            textvariable=self.button_text,
            padx=20,
            command=self.pressed_callback,
        ).pack(anchor=tk.W)
        self.button_text.set(self.button_state.get_text())

    def pressed_callback(self):
        if self.button_state.get_state() == RecordButtonState.State.START:
            rospy.loginfo("Start recording")
            now = datetime.datetime.now()
            datestr = now.strftime("%Y-%m-%dT%H-%M-%S")
            name = f"{get_robot()}_{datestr}"
            for manager in self.svo_service_managers:
                manager.start(name)
            self.bag_manager.start(name)
        else:
            rospy.loginfo("Stop recording")
            for manager in self.svo_service_managers:
                manager.stop()
            self.bag_manager.stop()

    def start_callback(self, path: str) -> None:
        self.button_state.set_stop()
        self.button_text.set(self.button_state.get_text())

    def stop_callback(self, path: str) -> None:
        self.button_state.set_start()
        self.button_text.set(self.button_state.get_text())
