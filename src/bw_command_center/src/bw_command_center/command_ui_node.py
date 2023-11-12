#!/usr/bin/env python
import tkinter as tk
from typing import List, Type

import rospy

from bw_command_center.command_ui.cage_corner_ui import CageCornerUI
from bw_command_center.command_ui.mode_ui import ModeUI
from bw_command_center.command_ui.ui_base import UiBase


def main():
    rospy.init_node("bw_command_center")
    window = tk.Tk()
    window.tk.call("tk", "scaling", 2.0)
    window.wm_title("BWBots Command Center")

    panels: List[Type[UiBase]] = [CageCornerUI, ModeUI]

    for panel in panels:
        frame = tk.Frame(master=window)
        panel(frame).pack()
        frame.pack()

    window.mainloop()


if __name__ == "__main__":
    main()
