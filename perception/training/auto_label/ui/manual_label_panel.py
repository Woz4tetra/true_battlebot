import logging
import tkinter as tk
from pathlib import Path
from tkinter import Tk, filedialog, ttk

import cv2
from matplotlib import pyplot as plt
from perception_tools.messages.image import Encoding, Image
from PIL import Image as PILImage
from PIL import ImageTk

from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.backend.video_source_collection import ALLOWED_VIDEO_EXTENSIONS
from auto_label.ui.canvas_image import CanvasImage
from auto_label.ui.panel import Panel


class ManualLabelPanel(Panel):
    def __init__(self, window: Tk, backend: ManualLabelBackend) -> None:
        super().__init__(window)
        self.backend = backend
        self.jump_count = 1

        self.shown_tk_image: ImageTk.PhotoImage | None = None
        self.image_frame = ttk.Frame(self.window)
        self.image_frame.rowconfigure(0, weight=1)
        self.image_frame.columnconfigure(0, weight=1)
        self.add_video_button = ttk.Button(self.window, text="Add Video", command=self.add_video_button_callback)
        self.next_frame_button = ttk.Button(self.window, text="Next Frame", command=self.next_frame_button_callback)
        self.prev_frame_button = ttk.Button(self.window, text="Previous Frame", command=self.prev_frame_button_callback)
        self.available_videos_frame = tk.Frame(self.window)
        self.image_canvas = CanvasImage(self.image_frame)

        self.logger = logging.getLogger(self.__class__.__name__)

    def pack(self) -> None:
        self.image_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.available_videos_frame.pack()
        self.add_video_button.pack(pady=10)
        self.next_frame_button.pack(pady=10)
        self.prev_frame_button.pack(pady=10)
        for video_name in self.backend.video_source.list_videos():
            self.add_video_button_to_frame(video_name)

        # Bind events to the Canvas
        self.window.bind("<Configure>", self.image_canvas.show_image_callback)  # canvas is resized
        self.window.bind("<ButtonPress-1>", self.image_canvas.move_from_callback)  # remember canvas position
        self.window.bind("<B1-Motion>", self.image_canvas.move_to_callback)  # move canvas to the new position
        self.window.bind("<MouseWheel>", self.image_canvas.wheel_callback)  # zoom for Windows and MacOS, but not Linux
        self.window.bind("<Button-5>", self.image_canvas.wheel_callback)  # zoom for Linux, wheel scroll down
        self.window.bind("<Button-4>", self.image_canvas.wheel_callback)  # zoom for Linux, wheel scroll up

        # Handle keystrokes in idle mode, because program slows down on a weak computers,
        # when too many key stroke events in the same time
        self.window.bind("<Key>", lambda event: self.window.after_idle(self.keystroke_callback, event))

    def add_video_button_callback(self) -> None:
        # Open file dialog to select video
        video_path = filedialog.askopenfilename(
            initialdir=str(self.backend.root_path),
            title="Select Video File",
            filetypes=[("Video Files", ".*")],
        )
        if video_path:
            video_path_obj = Path(video_path)
            self.backend.add_video(video_path_obj)
            self.add_video_button_to_frame(video_path_obj.name)
            self.next_frame_button_callback()
            self.logger.info(f"Video {video_path} added.")
        else:
            self.logger.warning("No video selected.")

    def add_video_button_to_frame(self, video_name: str) -> None:
        video_button = ttk.Button(self.available_videos_frame, text=video_name)
        video_button.pack(pady=10)
        video_button.bind("<Button-1>", lambda event: self.select_video_button_callback(video_name))
        self.logger.info(f"Video {video_name} added to frame.")

    def select_video_button_callback(self, video_name: str) -> None:
        self.backend.select_video(video_name)
        self.logger.info(f"Video {video_name} selected.")
        self.next_frame_button_callback()

    def next_frame_button_callback(self) -> None:
        self.logger.debug(f"Next frame button clicked. Jumping {self.jump_count} frames.")
        image = self.backend.next_frame(jump_count=self.jump_count)
        if image is not None:
            self.display_image(image)
        else:
            self.logger.warning("No more frames available.")

    def prev_frame_button_callback(self) -> None:
        self.logger.debug(f"Prev frame button clicked. Jumping {self.jump_count} frames.")
        image = self.backend.next_frame(jump_count=-1 * self.jump_count)
        if image is not None:
            self.display_image(image)
        else:
            self.logger.warning("No more frames available.")

    def display_image(self, image: Image) -> None:
        self.logger.debug(f"Displaying image {image.header.seq}")
        if image.encoding == Encoding.BGR8:
            image_array = cv2.cvtColor(image.data, cv2.COLOR_BGR2RGB)
        elif image.encoding == Encoding.RGB8:
            image_array = image.data
        else:
            self.logger.error(f"Unsupported image encoding: {image.encoding}")
            return
        # Convert the image to a format Tkinter can use
        self.image_canvas.set_image(PILImage.fromarray(image_array))

    def keystroke_callback(self, event: tk.Event) -> None:
        if event.keysym == "Left":
            self.prev_frame_button_callback()
        elif event.keysym == "Right":
            self.next_frame_button_callback()
        elif event.keysym == "Up":
            self.jump_count += 1
            self.logger.debug(f"Jump count increased to {self.jump_count}.")
        elif event.keysym == "Down":
            if self.jump_count > 1:
                self.jump_count -= 1
                self.logger.debug(f"Jump count decreased to {self.jump_count}.")
            else:
                self.logger.debug("Jump count is already at minimum.")
        elif event.keysym == "Escape":
            self.window.quit()
        else:
            self.image_canvas.keystroke_callback(event)
