import logging
import tkinter as tk
from pathlib import Path

import sv_ttk
from perception_tools.directories.config_directory import ConfigType, load_config_as_dict
from perception_tools.directories.data_directory import get_data_directory
from perception_tools.initialize_logger import initialize
from perception_tools.training.keypoints_config import load_keypoints_config

from auto_label.ai_interpolator.interpolation_process_manager import InterpolationProcessManager
from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.command_line_args import CommandLineArgs
from auto_label.config.auto_label_config import AutoLabelConfig
from auto_label.ui.manual_label_panel import ManualLabelPanel


def load_config(config_name: str) -> AutoLabelConfig:
    config_dict = load_config_as_dict(config_name, ConfigType.AUTO_LABEL)
    return AutoLabelConfig.from_dict(config_dict)


class App:
    def __init__(self, args: CommandLineArgs) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config = load_config(args.config)
        self.keypoints_config = load_keypoints_config(args.keypoints_config)
        self.auto_fill_inbetween = args.fill
        self.manual_label_backend = ManualLabelBackend(Path(self.config.data_root_directory))
        self.interpolation_process = InterpolationProcessManager(
            self.config, self.keypoints_config, self.manual_label_backend
        )
        initialize(self.config.log_level)

    def run_ui(self) -> None:
        self.window = tk.Tk()
        screen_width, screen_height = self.get_screen_size()
        ui_scale = 2.0 if (screen_width > 2000 or screen_height > 1500) else 1.0
        self.window.tk.call("tk", "scaling", 1.0)

        self.window.title("Auto Label Tool")
        default_size = self.config.default_size if self.config.default_size else (screen_width, screen_height)
        self.logger.info(f"Setting window size to {default_size}. UI scale: {ui_scale}")
        self.window.geometry(f"{default_size[0]}x{default_size[1]}")
        self.window.resizable(True, True)
        self.window.minsize(self.config.min_size[0], self.config.min_size[1])
        self.window.maxsize(self.config.max_size[0], self.config.max_size[1])

        self.set_icon()

        panel = ManualLabelPanel(
            self.window,
            self.config.default_jump_count,
            self.manual_label_backend,
            self.keypoints_config,
            self.interpolation_process,
            ui_scale,
        )
        panel.pack()

        sv_ttk.set_theme("light")
        self.window.mainloop()

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
        if self.auto_fill_inbetween:
            self.logger.info("Auto fill in between is enabled")
        else:
            self.run_ui()
