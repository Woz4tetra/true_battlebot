# from https://stackoverflow.com/questions/41656176/tkinter-canvas-zoom-move-pan
import logging
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk
from typing import Any

from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage
from PIL import Image, ImageTk


class DrawRectangleState:
    def __init__(self, canvas: tk.Canvas) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.canvas = canvas
        self.start_x = 0.0
        self.start_y = 0.0
        self.unscaled_start_x = 0.0
        self.unscaled_start_y = 0.0
        self.end_x = 0.0
        self.end_y = 0.0
        self.is_drawing = False
        self.is_cleared = True
        self.saved_bbox = (0.0, 0.0, 0.0, 0.0)
        self.rectangle_id = self.canvas.create_rectangle(0.0, 0.0, 0.0, 0.0, outline="red", width=2)
        self.hide()

    def hide(self) -> None:
        self.canvas.itemconfigure(self.rectangle_id, state="hidden")

    def show(self) -> None:
        self.canvas.itemconfigure(self.rectangle_id, state="normal")

    def start_drawing(self, start_x: float, start_y: float) -> None:
        self.start_x = start_x
        self.start_y = start_y
        self.is_drawing = True
        self.logger.debug(f"Start drawing rectangle at ({self.start_x}, {self.start_y})")
        self.show()

    def update_rectangle(self, canvas_x: float, canvas_y: float) -> None:
        if not self.is_drawing:
            return
        self.end_x = canvas_x
        self.end_y = canvas_y
        self.canvas.coords(self.rectangle_id, self.start_x, self.start_y, self.end_x, self.end_y)

    def finish_drawing(self) -> None:
        self.saved_bbox = (self.start_x, self.start_y, self.end_x, self.end_y)
        self.is_drawing = False
        self.logger.debug(f"Finish drawing rectangle at ({self.end_x}, {self.end_y})")
        self.hide()
        self.canvas.coords(self.rectangle_id, 0.0, 0.0, 0.0, 0.0)  # hide rectangle
        self.is_cleared = False

    def get_bbox(self) -> tuple[float, float, float, float] | None:
        """Get bounding box of the rectangle"""
        if self.is_cleared:
            return None
        return self.saved_bbox

    def clear(self) -> None:
        if self.is_cleared and not self.is_drawing:
            return
        self.finish_drawing()
        self.is_cleared = True


@dataclass
class AnnotationObjectIds:
    rectangle: int
    text: int
    keypoint_circles: list[int]
    keypoint_text: list[int]
    lines: list[int]

    def all_ids(self) -> list[int]:
        return (
            [
                self.rectangle,
                self.text,
            ]
            + self.keypoint_circles
            + self.keypoint_text
            + self.lines
        )


class CanvasImage:
    """Display and zoom image"""

    def __init__(self, frame: ttk.Frame, label_map: list[str], keypoint_names: dict[str, list[str]]) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._delta = 1.3  # zoom magnitude
        self._filter = Image.Resampling.NEAREST  # could be: NEAREST, BOX, BILINEAR, HAMMING, BICUBIC, LANCZOS
        self._previous_state = 0  # previous state of the keyboard
        self._image_set = False

        # Create ImageFrame in placeholder widget
        self._imframe = frame

        # Container for the image
        self.container: int | None = None

        # Vertical and horizontal scrollbars for canvas
        self.hbar = ttk.Scrollbar(self._imframe, orient="horizontal")
        self.vbar = ttk.Scrollbar(self._imframe, orient="vertical")
        self.hbar.grid(row=1, column=0, sticky="we")
        self.vbar.grid(row=0, column=1, sticky="ns")

        self.canvas_offset = 0.0, 0.0
        self.canvas_scale = 1.0, 1.0

        # Create canvas and bind it with scrollbars. Public for outer classes
        self.canvas = tk.Canvas(
            self._imframe, highlightthickness=0, xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set
        )
        self.canvas.grid(row=0, column=0, sticky="nswe")
        self.canvas.update()  # wait till canvas is created
        self.hbar.configure(command=self._scroll_x)  # bind scrollbars to the canvas
        self.vbar.configure(command=self._scroll_y)

        self.h_line1_id = self.canvas.create_line(0.0, 0.0, 0.0, 0.0, fill="white", width=1)  # horizontal line
        self.v_line1_id = self.canvas.create_line(0.0, 0.0, 0.0, 0.0, fill="white", width=1)  # vertical line

        self.h_line2_id = self.canvas.create_line(0.0, 0.0, 0.0, 0.0, fill="black", width=1)  # horizontal line
        self.v_line2_id = self.canvas.create_line(0.0, 0.0, 0.0, 0.0, fill="black", width=1)  # vertical line

        self.label_colors = ["red", "blue", "green", "purple", "pink", "yellow", "orange", "brown", "gray", "black"]
        self.label_map: list[str] = label_map

        self.keypoint_colors = ["orange", "blue", "green", "purple", "pink", "yellow"]
        self.keypoint_radius = 10
        self.keypoint_names: dict[str, list[str]] = keypoint_names

        self.selected_keypoint_ids: tuple[int, int] | None = None
        self.selected_keypoint_coords: tuple[float, float] | None = None

        # Bind events to the Canvas
        self.canvas.bind("<Configure>", self.show_image_callback)  # canvas is resized
        self.canvas.bind("<ButtonPress-3>", self.move_from_callback)  # remember canvas position
        self.canvas.bind("<B3-Motion>", self.move_to_callback)  # move canvas to the new position
        self.canvas.bind("<MouseWheel>", self.wheel_callback)  # zoom for Windows and MacOS, but not Linux
        self.canvas.bind("<Button-5>", self.wheel_callback)  # zoom for Linux, wheel scroll down
        self.canvas.bind("<Button-4>", self.wheel_callback)  # zoom for Linux, wheel scroll up
        self.canvas.bind("<Motion>", self.move_callback)
        self.canvas.bind("<ButtonPress-1>", self.button1_click_callback)

        self.draw_rectangle = DrawRectangleState(self.canvas)
        self.highlight_rectangle = self.canvas.create_rectangle(
            0.0, 0.0, 0.0, 0.0, outline="red", width=10, state="hidden"
        )

        self.annotation_ids: dict[int, AnnotationObjectIds] = {}  # annotations to canvas ids

    def set_image(self, image: Image.Image) -> None:
        self.logger.debug("Set image")
        self._image = image
        self._width, self._height = image.size  # get image size

        # Create image pyramid
        # Set ratio coefficient for image pyramid
        self._scale = 1.0  # scale for the canvas image zoom, public for outer classes
        self.canvas_offset = 0.0, 0.0
        self.canvas_scale = 1.0, 1.0

        # Decide if this image huge or not
        width, height = self._image.size  # public for outer classes

        if self.container is not None:
            self.canvas.delete(self.container)

        # Put image into container rectangle and use it to set proper coordinates to the image
        self.container = self.canvas.create_rectangle((0, 0, width, height), width=0)

        self._image_set = True
        self.redraw_figures()  # method for child classes
        self._show_image()  # show image on the canvas
        self.canvas.focus_set()  # set focus on the canvas
        self.draw_rectangle.clear()
        self._erase_annotation()

    def set_annotation(self, annotation: YoloKeypointImage) -> None:
        """Set annotation to the image"""
        self.logger.debug(f"Set annotation: {annotation}")
        self._annotation = annotation
        self._draw_annotation(self._annotation)

    def highlight_annotation(self, label_index: int) -> None:
        """Highlight annotation on the canvas"""
        self.logger.debug(f"Highlight annotation: {label_index}")
        annotation = self.annotation_ids[label_index]
        rectangle_coords = self.canvas.coords(annotation.rectangle)
        self.canvas.coords(self.highlight_rectangle, *rectangle_coords)
        self.canvas.itemconfigure(self.highlight_rectangle, state="normal")

    def hide_highlight(self) -> None:
        """Hide highlight rectangle"""
        self.canvas.itemconfigure(self.highlight_rectangle, state="hidden")

    def get_annotation(self) -> YoloKeypointImage:
        return self._annotation

    def _reset_canvas(self) -> None:
        """Move canvas to the initial position"""
        self.set_image(self._image)
        self._draw_annotation(self._annotation)

    def redraw_figures(self) -> None:
        """Dummy function to redraw figures in the children classes"""
        pass

    def grid(self, **kw: Any) -> None:
        """Put CanvasImage widget on the parent widget"""
        self._imframe.grid(**kw)  # place CanvasImage widget on the grid
        self._imframe.grid(sticky="nswe")  # make frame container sticky
        self._imframe.rowconfigure(0, weight=1)  # make canvas expandable
        self._imframe.columnconfigure(0, weight=1)

    def pack(self, **kw: Any) -> None:
        """Exception: cannot use pack with this widget"""
        raise Exception("Cannot use pack with the widget " + self.__class__.__name__)

    def place(self, **kw: Any) -> None:
        """Exception: cannot use place with this widget"""
        raise Exception("Cannot use place with the widget " + self.__class__.__name__)

    def _scroll_x(self, *args: Any, **kwargs: Any) -> None:
        """Scroll canvas horizontally and redraw the image"""
        self.canvas.xview(*args)  # scroll horizontally
        self._show_image()  # redraw the image

    def _scroll_y(self, *args: Any, **kwargs: Any) -> None:
        """Scroll canvas vertically and redraw the image"""
        self.canvas.yview(*args)  # scroll vertically
        self._show_image()  # redraw the image

    def show_image_callback(self, event: tk.Event) -> None:
        self._show_image()

    def _show_image(self) -> None:
        """Show image on the Canvas. Implements correct image zoom almost like in Google Maps"""
        if not self._image_set:
            return
        assert self.container is not None
        box_image = self.canvas.coords(self.container)  # get image area
        box_canvas = (
            self.canvas.canvasx(0),  # get visible area of the canvas
            self.canvas.canvasy(0),
            self.canvas.canvasx(self.canvas.winfo_width()),
            self.canvas.canvasy(self.canvas.winfo_height()),
        )
        box_img_int = tuple(map(int, box_image))  # convert to integer or it will not work properly

        # Get scroll region box
        box_scroll = [
            min(box_img_int[0], box_canvas[0]),
            min(box_img_int[1], box_canvas[1]),
            max(box_img_int[2], box_canvas[2]),
            max(box_img_int[3], box_canvas[3]),
        ]

        # Horizontal part of the image is in the visible area
        if box_scroll[0] == box_canvas[0] and box_scroll[2] == box_canvas[2]:
            box_scroll[0] = box_img_int[0]
            box_scroll[2] = box_img_int[2]

        # Vertical part of the image is in the visible area
        if box_scroll[1] == box_canvas[1] and box_scroll[3] == box_canvas[3]:
            box_scroll[1] = box_img_int[1]
            box_scroll[3] = box_img_int[3]

        # Convert scroll region to tuple and to integer
        self.canvas.configure(scrollregion=tuple(map(int, box_scroll)))  # type: ignore
        x1 = max(box_canvas[0] - box_image[0], 0)  # get coordinates (x1,y1,x2,y2) of the image tile
        y1 = max(box_canvas[1] - box_image[1], 0)
        x2 = min(box_canvas[2], box_image[2]) - box_image[0]
        y2 = min(box_canvas[3], box_image[3]) - box_image[1]
        if int(x2 - x1) > 0 and int(y2 - y1) > 0:  # show image if it's in the visible area
            image = self._image.crop(
                (int(x1 / self._scale), int(y1 / self._scale), int(x2 / self._scale), int(y2 / self._scale))
            )
            imagetk = ImageTk.PhotoImage(image.resize((int(x2 - x1), int(y2 - y1)), self._filter))
            imageid = self.canvas.create_image(
                max(box_canvas[0], box_img_int[0]), max(box_canvas[1], box_img_int[1]), anchor="nw", image=imagetk
            )
            self.canvas.lower(imageid)  # set image into background
            self._cropped_image = imagetk  # keep an extra reference to prevent garbage-collection

    def move_from_callback(self, event: tk.Event) -> None:
        """Remember previous coordinates for scrolling with the mouse"""
        self.canvas.scan_mark(event.x, event.y)

    def move_to_callback(self, event: tk.Event) -> None:
        """Drag (move) canvas to the new position"""
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        self._show_image()  # zoom tile and show it on the canvas

    def move_callback(self, event: tk.Event) -> None:
        """Draw crosshair on the canvas"""
        if self.container is None:
            return
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)

        bbox = self.canvas.coords(self.container)
        self.canvas.coords(self.h_line1_id, bbox[0], canvas_y, bbox[2], canvas_y)  # horizontal line
        self.canvas.coords(self.v_line1_id, canvas_x, bbox[1], canvas_x, bbox[3])  # vertical line

        self.canvas.coords(self.h_line2_id, bbox[0], canvas_y + 1, bbox[2], canvas_y + 1)  # horizontal line
        self.canvas.coords(self.v_line2_id, canvas_x + 1, bbox[1], canvas_x + 1, bbox[3])  # vertical line

        if self.selected_keypoint_ids:
            label_index, keypoint_index = self.selected_keypoint_ids
            self.update_selected_keypoint(label_index, keypoint_index, canvas_x, canvas_y)
        else:
            self.draw_rectangle.update_rectangle(canvas_x, canvas_y)  # update draw rectangle

    def update_selected_keypoint(self, label_index: int, keypoint_index: int, canvas_x: float, canvas_y: float) -> None:
        # move selected keypoint
        label = self._annotation.labels[label_index]
        image_x = self._x_scaled_canvas_to_norm(canvas_x)
        image_y = self._y_scaled_canvas_to_norm(canvas_y)
        self.selected_keypoint_coords = image_x, image_y
        r_scaled = self.keypoint_radius * self._scale
        self.canvas.coords(
            self.annotation_ids[label_index].keypoint_circles[keypoint_index],
            canvas_x - r_scaled,
            canvas_y - r_scaled,
            canvas_x + r_scaled,
            canvas_y + r_scaled,
        )
        self.canvas.coords(self.annotation_ids[label_index].keypoint_text[keypoint_index], canvas_x, canvas_y)
        # update lines
        prev_index = keypoint_index - 1
        if prev_index >= 0:
            self.canvas.coords(
                self.annotation_ids[label_index].lines[prev_index],
                self._x_norm_to_scaled_canvas(self.selected_keypoint_coords[0]),
                self._y_norm_to_scaled_canvas(self.selected_keypoint_coords[1]),
                self._x_norm_to_scaled_canvas(label.keypoints[prev_index][0]),
                self._y_norm_to_scaled_canvas(label.keypoints[prev_index][1]),
            )
        next_index = keypoint_index + 1
        if next_index < len(label.keypoints):
            self.canvas.coords(
                self.annotation_ids[label_index].lines[keypoint_index],
                self._x_norm_to_scaled_canvas(self.selected_keypoint_coords[0]),
                self._y_norm_to_scaled_canvas(self.selected_keypoint_coords[1]),
                self._x_norm_to_scaled_canvas(label.keypoints[next_index][0]),
                self._y_norm_to_scaled_canvas(label.keypoints[next_index][1]),
            )

    def outside(self, x: int, y: int) -> bool:
        """Checks if the point (x,y) is outside the image area"""
        assert self.container is not None
        bbox = self.canvas.coords(self.container)  # get image area
        return not (bbox[0] < x < bbox[2] and bbox[1] < y < bbox[3])

    def button1_click_callback(self, event: tk.Event) -> None:
        if not self._image_set:
            return
        canvas_x = self.canvas.canvasx(event.x)  # get coordinates of the event on the canvas
        canvas_y = self.canvas.canvasy(event.y)

        if self.selected_keypoint_ids is None:
            if keypoint_result := self.get_keypoint_under_cursor(canvas_x, canvas_y):
                self.selected_keypoint_ids = keypoint_result
                return
        else:
            self._update_annotation_with_selected_keypoint()
            self.selected_keypoint_ids = None
            self.selected_keypoint_coords = None
            return

        if self.outside(canvas_x, canvas_y):
            return
        if not self.draw_rectangle.is_drawing:
            self.draw_rectangle.start_drawing(canvas_x, canvas_y)  # start drawing rectangle
        else:
            self.draw_rectangle.finish_drawing()

    def _update_annotation_with_selected_keypoint(self) -> None:
        """Update annotation with selected keypoint"""
        if self.selected_keypoint_ids is None or self.selected_keypoint_coords is None:
            return
        label_index, keypoint_index = self.selected_keypoint_ids
        label = self._annotation.labels[label_index]
        label.keypoints[keypoint_index] = (
            self.selected_keypoint_coords[0],
            self.selected_keypoint_coords[1],
            label.keypoints[keypoint_index][2],
        )

    def get_keypoint_under_cursor(self, x: int, y: int) -> tuple[int, int] | None:
        """Get keypoint under cursor"""
        if not self._image_set:
            return None
        r_scaled = self.keypoint_radius * self._scale
        for label_index in self.annotation_ids.keys():
            for keypoint_index, keypoint in enumerate(self._annotation.labels[label_index].keypoints):
                canvas_x = self._x_norm_to_scaled_canvas(keypoint[0])
                canvas_y = self._y_norm_to_scaled_canvas(keypoint[1])
                if abs(canvas_x - x) < r_scaled and abs(canvas_y - y) < r_scaled:
                    return label_index, keypoint_index
        return None

    def get_bbox(self) -> tuple[float, float, float, float] | None:
        """Get bounding box of the rectangle"""
        if not self._image_set:
            return None
        bbox = self.draw_rectangle.get_bbox()
        if bbox is not None:
            bbox = self._bbox_scaled_canvas_to_norm(bbox)
            self.logger.debug(f"Get bounding box: {bbox}")
            self.draw_rectangle.clear()
        return bbox

    def wheel_callback(self, event: tk.Event) -> None:
        """Zoom with mouse wheel"""
        if not self._image_set:
            return
        x = self.canvas.canvasx(event.x)  # get coordinates of the event on the canvas
        y = self.canvas.canvasy(event.y)
        if self.outside(x, y):
            return  # zoom only inside image area
        relative_scale = 1.0
        # Respond to Linux (event.num) or Windows (event.delta) wheel event
        if event.num == 5 or event.delta == -120:  # scroll down, smaller
            relative_scale /= self._delta
        if event.num == 4 or event.delta == 120:  # scroll up, bigger
            relative_scale *= self._delta
        new_scale = self._scale * relative_scale
        if new_scale < 0.5:  # don't zoom out too much
            return
        if new_scale > 10.0:  # don't zoom in too much
            return
        self._scale = new_scale

        if relative_scale != 1.0:
            self._set_position_and_scale(x, y, relative_scale)  # set new scale

    def _set_position_and_scale(self, focal_x: float, focal_y: float, relative_scale: float) -> None:
        self.canvas.scale("all", focal_x, focal_y, relative_scale, relative_scale)  # rescale all objects
        self.canvas_offset = (
            self.canvas_offset[0] * relative_scale + focal_x * (1 - relative_scale),
            self.canvas_offset[1] * relative_scale + focal_y * (1 - relative_scale),
        )
        self.canvas_scale = (
            self.canvas_scale[0] * relative_scale,
            self.canvas_scale[1] * relative_scale,
        )
        # Redraw some figures before showing image on the screen
        self.redraw_figures()  # method for child classes
        self._show_image()

    def _x_scaled_canvas_to_image(self, x: float) -> float:
        return (x - self.canvas_offset[0]) / self.canvas_scale[0]

    def _y_scaled_canvas_to_image(self, y: float) -> float:
        return (y - self.canvas_offset[1]) / self.canvas_scale[1]

    def _x_scaled_canvas_to_norm(self, x: float) -> float:
        return self._x_scaled_canvas_to_image(x) / self._width

    def _y_scaled_canvas_to_norm(self, y: float) -> float:
        return self._y_scaled_canvas_to_image(y) / self._height

    def _x_image_to_scaled_canvas(self, x: float) -> float:
        return x * self.canvas_scale[0] + self.canvas_offset[0]

    def _y_image_to_scaled_canvas(self, y: float) -> float:
        return y * self.canvas_scale[1] + self.canvas_offset[1]

    def _x_norm_to_scaled_canvas(self, x: float) -> float:
        return self._x_image_to_scaled_canvas(x * self._width)

    def _y_norm_to_scaled_canvas(self, y: float) -> float:
        return self._y_image_to_scaled_canvas(y * self._height)

    def _bbox_scaled_canvas_to_norm(self, bbox: tuple[float, ...]) -> tuple[float, float, float, float]:
        """Convert coordinates from scaled canvas to normalized image (0..1)"""
        x0, y0, x1, y1 = bbox
        x0_new = self._x_scaled_canvas_to_norm(x0)
        y0_new = self._y_scaled_canvas_to_norm(y0)
        x1_new = self._x_scaled_canvas_to_norm(x1)
        y1_new = self._y_scaled_canvas_to_norm(y1)
        return x0_new, y0_new, x1_new, y1_new

    def _bbox_norm_to_scaled_canvas(self, bbox: tuple[float, ...]) -> tuple[float, float, float, float]:
        """Convert coordinates from normalized image (0..1) to scaled canvas coordinates"""
        x0, y0, x1, y1 = bbox
        x0_new = self._x_norm_to_scaled_canvas(x0)
        y0_new = self._y_norm_to_scaled_canvas(y0)
        x1_new = self._x_norm_to_scaled_canvas(x1)
        y1_new = self._y_norm_to_scaled_canvas(y1)
        return x0_new, y0_new, x1_new, y1_new

    def _bbox_image_to_scaled_canvas(self, bbox: tuple[float, ...]) -> tuple[float, float, float, float]:
        """Convert coordinates from image pixels to scaled canvas coordinates"""
        x0, y0, x1, y1 = bbox
        x0_new = self._x_image_to_scaled_canvas(x0)
        y0_new = self._y_image_to_scaled_canvas(y0)
        x1_new = self._x_image_to_scaled_canvas(x1)
        y1_new = self._y_image_to_scaled_canvas(y1)
        return x0_new, y0_new, x1_new, y1_new

    def keystroke_callback(self, event: tk.Event) -> None:
        """Scrolling with the keyboard.
        Independent from the language of the keyboard, CapsLock, <Ctrl>+<key>, etc."""
        if not isinstance(event.state, int):
            return
        if event.state - self._previous_state == 4:  # means that the Control key is pressed
            return
        self._previous_state = event.state  # remember the last keystroke state
        # Up, Down, Left, Right keystrokes
        if event.keysym == "l":
            self._scroll_x("scroll", 1, "unit", event=event)
        elif event.keysym == "j":
            self._scroll_x("scroll", -1, "unit", event=event)
        elif event.keysym == "i":
            self._scroll_y("scroll", -1, "unit", event=event)
        elif event.keysym == "k":
            self._scroll_y("scroll", 1, "unit", event=event)
        elif event.keysym == "Escape":
            self.draw_rectangle.clear()
            self._reset_selected_keypoint()
        elif event.keysym == "r":
            self._reset_canvas()
            self._show_image()

    def _draw_annotation(self, annotation: YoloKeypointImage) -> None:
        """Draw annotation on the canvas"""
        if not self._image_set:
            return
        assert self.container is not None
        for label_index in self.annotation_ids:
            for obj_id in self.annotation_ids[label_index].all_ids():
                self.canvas.delete(obj_id)
        self.annotation_ids = {}
        for label_index, label in enumerate(annotation.labels):
            self.annotation_ids[label_index] = self._draw_label(label)

    def _draw_label(self, label: YoloKeypointAnnotation) -> AnnotationObjectIds:
        """Draw label on the canvas"""
        assert self.container is not None
        canvas_border = tuple(map(int, self.canvas.coords(self.container)))
        bbox = label.corners
        bbox = self._bbox_norm_to_scaled_canvas(bbox)
        x0 = max(int(bbox[0]), canvas_border[0])
        y0 = max(int(bbox[1]), canvas_border[1])
        x1 = min(int(bbox[2]), canvas_border[2])
        y1 = min(int(bbox[3]), canvas_border[3])
        canvas_border_scaled = self._bbox_image_to_scaled_canvas(canvas_border)
        self.logger.debug(f"Draw label: {label} with bbox: {(x0, y0, x1, y1)}")
        color = self.label_colors[label.class_index % len(self.label_colors)]
        obj_ids = AnnotationObjectIds(
            rectangle=self.canvas.create_rectangle(x0, y0, x1, y1, outline=color, width=2),
            text=self.canvas.create_text(
                x0, y0 - 10, text=self._get_class_name(label.class_index), fill=color, font=("Arial", 12, "bold")
            ),
            keypoint_circles=[],
            keypoint_text=[],
            lines=[],
        )
        prev_x = None
        prev_y = None
        r = self.keypoint_radius
        for index, keypoint in enumerate(label.keypoints):
            color = self.keypoint_colors[index % len(self.keypoint_colors)]
            x = int(self._x_norm_to_scaled_canvas(keypoint[0]))
            y = int(self._y_norm_to_scaled_canvas(keypoint[1]))
            x = int(min(max(x, canvas_border_scaled[0]), canvas_border_scaled[2]))
            y = int(min(max(y, canvas_border_scaled[1]), canvas_border_scaled[3]))
            obj_ids.keypoint_circles.append(
                self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, outline=color, width=1)
            )
            obj_ids.keypoint_text.append(
                self.canvas.create_text(
                    x,
                    y,
                    text=self._get_keypoint_name(label.class_index, index),
                    fill=color,
                    font=("Arial", 12),
                )
            )
            if prev_x is not None and prev_y is not None:
                obj_ids.lines.append(self.canvas.create_line(x, y, prev_x, prev_y, fill="grey", width=2))
            prev_x = x
            prev_y = y
        return obj_ids

    def _get_class_name(self, class_index: int) -> str:
        """Get class name from the class index"""
        if class_index < len(self.label_map):
            return self.label_map[class_index]
        return str(class_index)

    def _get_keypoint_name(self, class_index: int, keypoint_index: int) -> str:
        """Get keypoint name from the class index and keypoint index"""
        if class_index >= len(self.label_map):
            return str(keypoint_index)
        class_name = self.label_map[class_index]
        if class_name not in self.keypoint_names:
            return str(keypoint_index)
        keypoint_names = self.keypoint_names[class_name]
        if keypoint_index >= len(keypoint_names):
            return str(keypoint_index)
        return keypoint_names[keypoint_index]

    def _reset_selected_keypoint(self) -> None:
        if self.selected_keypoint_ids is None:
            return
        self.logger.debug(f"Reset selected keypoint: {self.selected_keypoint_ids}")
        label_index, keypoint_index = self.selected_keypoint_ids

        # put keypoint back to the original position
        label = self._annotation.labels[label_index]
        image_x = label.keypoints[keypoint_index][0]
        image_y = label.keypoints[keypoint_index][1]
        canvas_x = self._x_norm_to_scaled_canvas(image_x)
        canvas_y = self._y_norm_to_scaled_canvas(image_y)
        self.update_selected_keypoint(label_index, keypoint_index, canvas_x, canvas_y)

        self.selected_keypoint_ids = None

    def _erase_annotation(self) -> None:
        """Erase annotation from the canvas"""
        for obj_ids in self.annotation_ids.values():
            for obj_id in obj_ids.all_ids():
                self.canvas.delete(obj_id)
        self.annotation_ids = {}
        self._annotation = YoloKeypointImage()

    def destroy(self) -> None:
        """ImageFrame destructor"""
        self._image.close()
        del self._image  # delete image variable
        self.canvas.destroy()
        self._imframe.destroy()
