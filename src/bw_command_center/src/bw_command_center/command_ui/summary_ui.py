import tkinter as tk

from bw_tools.environment import get_image_version, get_map, get_robot

from bw_command_center.command_ui.ui_base import UiBase


class SummaryUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        tk.Label(master=left_frame, text="Summary").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Robot: {get_robot()}").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Map: {get_map()}").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Version: {get_image_version()}").pack(anchor=tk.W)
