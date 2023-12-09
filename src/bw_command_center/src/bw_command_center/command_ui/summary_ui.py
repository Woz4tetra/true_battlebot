import tkinter as tk

from bw_tools.environment import get_system_info

from bw_command_center.command_ui.ui_base import UiBase


class SummaryUI(UiBase):
    def __init__(self, window) -> None:
        self.window = window

    def pack(self) -> None:
        left_frame = tk.Frame(master=self.window)
        left_frame.grid(row=0, column=0, sticky="W", padx=5, pady=5)
        right_frame = tk.Frame(master=self.window)
        right_frame.grid(row=0, column=2, sticky="W", padx=5, pady=5)

        info = get_system_info()
        tk.Label(master=left_frame, text="Summary").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Robot: {info.robot}").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Map: {info.map}").pack(anchor=tk.W)
        tk.Label(master=right_frame, text=f"Version: {info.version}").pack(anchor=tk.W)
