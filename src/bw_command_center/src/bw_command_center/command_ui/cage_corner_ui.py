import tkinter as tk

import rospy
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_tools.structs.cage_corner import CageCorner

from bw_command_center.command_ui.ui_base import UiBase


class CageCornerUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window
        self.selected_value = tk.IntVar()
        self.cage_corner_pub = rospy.Publisher("set_cage_corner", RosCageCorner, queue_size=1, latch=True)
        self.cage_corner_pub.publish(CageCorner.DOOR_SIDE.to_msg())

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        tk.Label(master=left_frame, text="Cage corner:").pack(anchor=tk.W)
        tk.Radiobutton(
            master=right_frame,
            text="Door side",
            padx=20,
            variable=self.selected_value,
            value=CageCorner.DOOR_SIDE.value,
            command=self.value_selected_callback,
        ).pack(anchor=tk.W)
        tk.Radiobutton(
            master=right_frame,
            text="Far side",
            padx=20,
            variable=self.selected_value,
            value=CageCorner.FAR_SIDE.value,
            command=self.value_selected_callback,
        ).pack(anchor=tk.W)

    def value_selected_callback(self):
        self.cage_corner_pub.publish(CageCorner(self.selected_value.get()).to_msg())
