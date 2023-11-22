import tkinter as tk

import rospy
from std_msgs.msg import Empty

from bw_command_center.command_ui.ui_base import UiBase


class RequestFieldUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window
        self.manual_plane_request = rospy.Publisher("manual_plane_request", Empty, queue_size=1, latch=True)

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        tk.Label(master=left_frame, text="Request field:").pack(anchor=tk.W)
        tk.Button(
            master=right_frame,
            text="Send",
            padx=20,
            command=self.pressed_callback,
        ).pack(anchor=tk.W)

    def pressed_callback(self):
        self.manual_plane_request.publish(Empty())
