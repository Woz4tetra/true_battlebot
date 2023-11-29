#!/usr/bin/env python
import os
import tkinter as tk
from typing import List, Type

import rospy
from bw_tools.typing import get_param

from bw_command_center.command_ui.cage_corner_ui import CageCornerUI
from bw_command_center.command_ui.mode_ui import ModeUI
from bw_command_center.command_ui.request_field_ui import RequestFieldUI
from bw_command_center.command_ui.request_record_ui import RequestRecordUI
from bw_command_center.command_ui.summary_ui import SummaryUI
from bw_command_center.command_ui.ui_base import UiBase


class App:
    def __init__(self) -> None:
        icon_path = get_param("~icon_path", "app.ico")

        self.window = tk.Tk()
        self.window.tk.call("tk", "scaling", 2.0)
        self.window.wm_title("BWBots Command Center")
        if os.path.exists(icon_path):
            if icon_path.endswith(".ico"):
                self.window.iconbitmap(icon_path)
            else:
                img = tk.PhotoImage(file=icon_path)
                self.window.tk.call("wm", "iconphoto", self.window._w, img)
        else:
            rospy.logwarn(f"Icon path {icon_path} does not exist")

        self.panels: List[Type[UiBase]] = [SummaryUI, CageCornerUI, ModeUI, RequestFieldUI, RequestRecordUI]

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
    rospy.init_node("bw_command_center", log_level=rospy.DEBUG)
    App().run()


if __name__ == "__main__":
    main()
