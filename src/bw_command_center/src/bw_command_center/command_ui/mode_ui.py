import tkinter as tk

import rospy
from bw_interfaces.msg import BehaviorMode as RosBehaviorMode
from bw_tools.structs.behavior_mode import BehaviorMode

from bw_command_center.command_ui.ui_base import UiBase


class ModeUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window
        self.selected_value = tk.IntVar()
        self.mode_pub = rospy.Publisher("behavior_mode", RosBehaviorMode, queue_size=1, latch=True)
        self.mode_pub.publish(BehaviorMode.IDLE.to_msg())

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        tk.Label(master=left_frame, text="Mode:").pack(anchor=tk.W)
        tk.Radiobutton(
            master=right_frame,
            text="Idle",
            padx=20,
            variable=self.selected_value,
            value=BehaviorMode.IDLE.value,
            command=self.value_selected_callback,
        ).pack(anchor=tk.W)
        tk.Radiobutton(
            master=right_frame,
            text="Go to corner",
            padx=20,
            variable=self.selected_value,
            value=BehaviorMode.CORNER.value,
            command=self.value_selected_callback,
        ).pack(anchor=tk.W)
        tk.Radiobutton(
            master=right_frame,
            text="Fight!!!",
            padx=20,
            variable=self.selected_value,
            value=BehaviorMode.FIGHT.value,
            command=self.value_selected_callback,
        ).pack(anchor=tk.W)

    def value_selected_callback(self):
        self.mode_pub.publish(BehaviorMode(self.selected_value.get()).to_msg())
