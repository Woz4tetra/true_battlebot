#!/usr/bin/env python
import os
import tkinter as tk
from threading import Thread
from typing import List, Type

import rospy

from bw_command_center.command_ui.cage_corner_ui import CageCornerUI
from bw_command_center.command_ui.mode_ui import ModeUI
from bw_command_center.command_ui.ui_base import UiBase


class App:
    def __init__(self) -> None:
        self.window = tk.Tk()
        self.window.tk.call("tk", "scaling", 2.0)
        self.window.wm_title("BWBots Command Center")

        self.panels: List[Type[UiBase]] = [CageCornerUI, ModeUI]

    def run(self) -> None:
        for panel in self.panels:
            frame = tk.Frame(master=self.window)
            panel(frame).pack()
            frame.pack()
        self.watcher_task()
        self.window.mainloop()

    def watcher_task(self):
        if rospy.is_shutdown():
            rospy.loginfo("Shutting down command center")
            os._exit(1)
        self.window.after(200, self.watcher_task)


def main():
    rospy.init_node("bw_command_center")
    App().run()


if __name__ == "__main__":
    main()
