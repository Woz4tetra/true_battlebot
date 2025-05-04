import logging
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import sv_ttk
from perception_tools.directories.config_directory import ConfigType, load_config_as_dict
from perception_tools.directories.data_directory import get_data_directory
from perception_tools.initialize_logger import initialize

from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.command_line_args import CommandLineArgs
from auto_label.config.auto_label_config import AutoLabelConfig
from auto_label.ui.manual_label_panel import ManualLabelPanel


def load_config(config_name: str) -> AutoLabelConfig:
    config_dict = load_config_as_dict(config_name, ConfigType.AUTO_LABEL)
    return AutoLabelConfig.from_dict(config_dict)


class App:
    def __init__(self, args: CommandLineArgs) -> None:
        self.config = load_config(args.config)
        initialize(self.config.log_level)
        self.window = tk.Tk()
        self.window.tk.call("tk", "scaling", 1.0)

        self.window.title("Auto Label Tool")
        default_size = self.config.default_size if self.config.default_size else self.get_screen_size()
        self.window.geometry(f"{default_size[0]}x{default_size[1]}")
        self.window.resizable(True, True)
        self.window.minsize(self.config.min_size[0], self.config.min_size[1])
        self.window.maxsize(self.config.max_size[0], self.config.max_size[1])

        self.set_icon()

        manual_label_backend = ManualLabelBackend(Path(self.config.data_root_directory))
        panels = [ManualLabelPanel(self.window, manual_label_backend)]
        for panel in panels:
            panel.pack()

        sv_ttk.set_theme("light")

        self.logger = logging.getLogger(self.__class__.__name__)

    def get_screen_size(self) -> tuple[int, int]:
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()
        return screen_width, screen_height

    def set_icon(self) -> None:
        icon_path = get_data_directory() / "images" / "bwbots.png"
        if icon_path.exists():
            img = tk.PhotoImage(file=icon_path)
            self.window.tk.call("wm", "iconphoto", self.window._w, img)  # type: ignore
        else:
            self.logger.warning(f"Icon path {icon_path} does not exist")

    def run(self) -> None:
        self.window.mainloop()
