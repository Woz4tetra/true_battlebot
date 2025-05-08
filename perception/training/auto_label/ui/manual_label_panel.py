import logging
import string
import tkinter as tk
from pathlib import Path
from tkinter import Tk, filedialog, ttk

import cv2
from perception_tools.messages.image import Encoding, Image
from perception_tools.training.keypoints_config import KeypointsConfig
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage
from PIL import Image as PILImage
from PIL import ImageTk

from auto_label.ai_interpolator.ai_interpolator import AiInterpolator
from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.ui.canvas_image import CanvasImage
from auto_label.ui.label_selection_toolbar import LabelSelectionToolbar
from auto_label.ui.label_table import LabelTable
from auto_label.ui.panel import Panel


class ManualLabelPanel(Panel):
    def __init__(
        self,
        window: Tk,
        default_jump_count: int,
        backend: ManualLabelBackend,
        keypoints_config: KeypointsConfig,
        ai_interpolator: AiInterpolator,
    ) -> None:
        super().__init__(window)
        self.backend = backend
        self.keypoints_config = keypoints_config
        self.jump_count = default_jump_count
        self.ai_interpolator = ai_interpolator

        self.shown_tk_image: ImageTk.PhotoImage | None = None

        self.window.rowconfigure(2, weight=1)
        self.window.columnconfigure(0, weight=1)

        self.edit_frame = ttk.Frame(self.window)
        self.edit_frame.rowconfigure(0, weight=1)
        self.edit_frame.columnconfigure(0, weight=1)
        self.edit_frame.columnconfigure(1, weight=9)
        self.image_display_frame = ttk.Frame(self.edit_frame)
        self.image_display_frame.rowconfigure(0, weight=1)
        self.image_display_frame.columnconfigure(0, weight=1)
        self.label_table_frame = ttk.Frame(self.edit_frame)
        self.label_table_frame.columnconfigure(0, weight=1)

        self.video_manage_frame = ttk.Frame(self.window)
        self.frame_number_label = ttk.Label(self.video_manage_frame, text="", width=15)
        self.skip_frame_label = ttk.Label(self.video_manage_frame, text="Skip frames:")
        self.skip_frame_entry = ttk.Entry(self.video_manage_frame, width=5)
        self.add_video_button = ttk.Button(
            self.video_manage_frame, text="Add Video", command=self.add_video_button_callback
        )
        self.next_frame_button = ttk.Button(
            self.video_manage_frame, text="Next Frame", command=self.next_frame_button_callback
        )
        self.prev_frame_button = ttk.Button(
            self.video_manage_frame, text="Previous Frame", command=self.prev_frame_button_callback
        )

        self.label_selection_frame = ttk.Frame(self.window)
        self.label_selection_toolbar = LabelSelectionToolbar(
            self.label_selection_frame, self.keypoints_config.labels, self.on_label_selected
        )

        self.selected_video_option = tk.StringVar(self.window)
        self.videos_dropdown = ttk.OptionMenu(self.video_manage_frame, self.selected_video_option)
        self.update_video_options()
        self.selected_video_option.set("Select a video")

        self.ai_interpolate_button = ttk.Button(
            self.video_manage_frame, text="Interpolate", command=self.interpolate_button_callback
        )

        self.image_canvas = CanvasImage(
            self.image_display_frame,
            [label.value for label in self.keypoints_config.labels],
            self.keypoints_config.keypoint_names,
        )
        self.label_table = LabelTable(
            self.label_table_frame,
            self.keypoints_config,
            self.delete_annotation_callback,
            self.highlight_annotation_callback,
            self.unhighlight_annotation_callback,
        )
        self.current_image: Image | None = None
        self.current_label_index = 0

        self.logger = logging.getLogger(self.__class__.__name__)

    def pack(self) -> None:
        self.edit_frame.grid(row=2, column=0, sticky="nsew")
        self.label_table_frame.grid(row=0, column=0, sticky="nsew")
        self.image_display_frame.grid(row=0, column=1, sticky="nsew")
        self.video_manage_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        self.frame_number_label.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        self.skip_frame_label.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        self.skip_frame_entry.grid(row=0, column=2, sticky="ew", padx=5, pady=5)
        self.prev_frame_button.grid(row=0, column=3, sticky="ew", padx=5, pady=5)
        self.next_frame_button.grid(row=0, column=4, sticky="ew", padx=5, pady=5)
        self.ai_interpolate_button.grid(row=0, column=5, sticky="ew", padx=5, pady=5)
        self.add_video_button.grid(row=0, column=6, sticky="ew", padx=5, pady=5)
        self.videos_dropdown.grid(row=0, column=7, sticky="ew", padx=5, pady=5)
        self.label_selection_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

        # Bind events
        self.window.bind_all("<Button-1>", lambda event: event.widget.focus_set())

        self.skip_frame_entry.bind("<Return>", self.skip_frame_entry_callback)
        self.update_skip_frame_entry()

        self.window.bind("<ButtonPress-1>", self.button1_click_callback)
        # Handle keystrokes in idle mode, because program slows down on a weak computers,
        # when too many key stroke events in the same time
        self.window.bind("<Key>", lambda event: self.window.after_idle(self.keystroke_callback, event))

    def update_video_options(self) -> None:
        menu = self.videos_dropdown["menu"]
        menu.delete(0, "end")
        menu.add_command(label="Select a video", command=lambda: None)
        for video_name in self.backend.video_source.list_videos():
            menu.add_command(
                label=video_name, command=lambda value=video_name: self.select_video_button_callback(value)
            )

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
            self.update_video_options()
            self.reload_image()
            self.logger.info(f"Video {video_path} added.")
        else:
            self.logger.warning("No video selected.")

    def select_video_button_callback(self, video_name: str) -> None:
        if video_name == self.backend.get_video_name():
            self.logger.debug(f"Video {video_name} is already selected.")
            return
        self.backend.select_video(video_name)
        self.logger.info(f"Video {video_name} selected.")
        self.selected_video_option.set(video_name)
        self.reload_image()

    def reload_image(self) -> None:
        if image := self.backend.next_frame():
            self.display_image(image)
        self.update_frame_number_label(image)

    def next_frame_button_callback(self) -> None:
        self.save_bbox_from_canvas()
        self.logger.debug(f"Next frame button clicked. Jumping {self.jump_count} frames.")
        image = self.backend.next_frame(jump_count=self.jump_count)
        self.update_frame_number_label(image)
        if image is not None:
            self.display_image(image)
        else:
            self.logger.warning("No more frames available.")

    def prev_frame_button_callback(self) -> None:
        self.save_bbox_from_canvas()
        self.logger.debug(f"Prev frame button clicked. Jumping {self.jump_count} frames.")
        image = self.backend.next_frame(jump_count=-1 * self.jump_count)
        self.update_frame_number_label(image)
        if image is not None:
            self.display_image(image)
        else:
            self.logger.warning("No more frames available.")

    def update_frame_number_label(self, image: Image | None) -> None:
        if self.backend.selected_video is None or image is None:
            self.frame_number_label.config(text="")
            return
        self.frame_number_label.config(
            text=f"Frame: {image.header.seq} / {self.backend.selected_video.get_num_frames()}"
        )

    def display_image(self, image: Image) -> None:
        self.logger.debug(f"Displaying image {image.header.seq}")
        if image.encoding == Encoding.BGR8:
            image_array = cv2.cvtColor(image.data, cv2.COLOR_BGR2RGB)
        elif image.encoding == Encoding.RGB8:
            image_array = image.data
        else:
            self.logger.error(f"Unsupported image encoding: {image.encoding}")
            return
        self.current_image = image
        annotation = self.get_current_annotation()
        # Convert the image to a format Tkinter can use
        self.image_canvas.set_image(PILImage.fromarray(image_array))
        self.update_annotation(annotation)

    def keystroke_callback(self, event: tk.Event) -> None:
        if event.keysym == "Left" or event.keysym == "a":
            self.prev_frame_button_callback()
        elif event.keysym == "Right" or event.keysym == "d":
            self.next_frame_button_callback()
        elif event.keysym == "Up" or event.keysym == "w":
            self.update_jump_count(self.jump_count + 1)
            self.update_skip_frame_entry()
        elif event.keysym == "Down" or event.keysym == "s":
            self.update_jump_count(self.jump_count - 1)
            self.update_skip_frame_entry()
        elif event.keysym in string.digits:
            index = int(event.keysym) - 1
            if index == -1:
                index = 10
            self.label_selection_toolbar.set_label(index)
        else:
            self.image_canvas.keystroke_callback(event)

    def update_jump_count(self, jump_count: int) -> None:
        if jump_count > 0:
            self.jump_count = jump_count
            self.logger.debug(f"Jump count updated to {self.jump_count}.")

        self.skip_frame_entry.delete(0, tk.END)
        self.skip_frame_entry.insert(0, str(self.jump_count))

    def update_skip_frame_entry(self) -> None:
        self.skip_frame_entry.delete(0, tk.END)
        self.skip_frame_entry.insert(0, str(self.jump_count))

    def skip_frame_entry_callback(self, event: tk.Event) -> None:
        self.update_jump_count(int(self.skip_frame_entry.get()))

        # unselect the entry
        self.skip_frame_entry.selection_clear()

    def button1_click_callback(self, event: tk.Event) -> None:
        self.save_bbox_from_canvas()

    def save_bbox_from_canvas(self) -> None:
        new_bbox = self.image_canvas.get_bbox()
        if self.current_image is None:
            self.logger.warning("No image is currently displayed. Cannot add annotation.")
            return
        if new_bbox is None:
            current_annotation = self.image_canvas.get_annotation()
            if not current_annotation.is_empty():
                self.backend.update_annotation(self.current_image, current_annotation)
            return
        self.logger.debug(f"Saving bbox {new_bbox} to backend.")
        annotation = self.backend.add_annotation(self.current_image, new_bbox, self.current_label_index)
        self.update_annotation(annotation)

    def update_annotation(self, annotation: YoloKeypointImage | None) -> None:
        if annotation is not None:
            self.label_table.populate_table(annotation)
            self.image_canvas.set_annotation(annotation)
        else:
            self.label_table.hide_table()

    def get_current_annotation(self) -> YoloKeypointImage | None:
        if self.current_image is None:
            self.logger.warning("No image is currently displayed. Cannot get annotation.")
            return None
        annotation = self.backend.annotations_cache.get_annotation(
            self.backend.get_video_name(), self.current_image.header.seq
        )
        if annotation is not None:
            return annotation
        return None

    def delete_annotation_callback(self, row_index: int) -> YoloKeypointImage | None:
        if self.current_image is None:
            self.logger.warning("No image is currently displayed. Cannot delete annotation.")
            return None
        annotation = self.get_current_annotation()
        if annotation is None:
            self.logger.warning("No annotation found. Cannot delete.")
            return None
        annotation.labels.pop(row_index)
        self.backend.update_annotation(self.current_image, annotation)
        self.update_annotation(annotation)
        return annotation

    def highlight_annotation_callback(self, row_index: int) -> None:
        if self.current_image is None:
            self.logger.warning("No image is currently displayed. Cannot highlight annotation.")
            return
        annotation = self.get_current_annotation()
        if annotation is None:
            self.logger.warning("No annotation found. Cannot highlight.")
            return
        self.image_canvas.highlight_annotation(row_index)

    def unhighlight_annotation_callback(self, row_index: int) -> None:
        if self.current_image is None:
            self.logger.warning("No image is currently displayed. Cannot unhighlight annotation.")
            return
        annotation = self.get_current_annotation()
        if annotation is None:
            self.logger.warning("No annotation found. Cannot unhighlight.")
            return
        self.image_canvas.hide_highlight()

    def on_label_selected(self, label_index: int) -> None:
        self.current_label_index = label_index

    def interpolate_button_callback(self) -> None:
        self.ai_interpolator.interpolate_from_current_frame()
