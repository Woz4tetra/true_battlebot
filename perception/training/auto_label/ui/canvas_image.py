# from https://stackoverflow.com/questions/41656176/tkinter-canvas-zoom-move-pan
import math
import tkinter as tk
from tkinter import ttk
from typing import Any

from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage
from PIL import Image, ImageTk


class HighlightRectangleState:
    def __init__(self, canvas: tk.Canvas) -> None:
        self.canvas = canvas
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0
        self.is_drawing = False
        self.rectangle_id = self.canvas.create_rectangle(0, 0, 0, 0, outline="red", width=2)
        self.hide()

    def hide(self) -> None:
        self.canvas.itemconfigure(self.rectangle_id, state="hidden")

    def show(self) -> None:
        self.canvas.itemconfigure(self.rectangle_id, state="normal")

    def start_drawing(self, event: tk.Event) -> None:
        self.start_x = event.x
        self.start_y = event.y
        self.is_drawing = True
        print(f"Start drawing rectangle at ({self.start_x}, {self.start_y})")
        self.show()

    def offset_position(self, x: int, y: int) -> None:
        if not self.is_drawing:
            return

    def update_rectangle(self, event: tk.Event) -> None:
        if not self.is_drawing:
            return
        self.end_x = event.x
        self.end_y = event.y
        self.canvas.coords(self.rectangle_id, self.start_x, self.start_y, self.end_x, self.end_y)

    def finish_drawing(self) -> None:
        self.is_drawing = False
        print(f"Finish drawing rectangle at ({self.end_x}, {self.end_y})")
        self.hide()
        self.canvas.coords(self.rectangle_id, 0, 0, 0, 0)


class CanvasImage:
    """Display and zoom image"""

    def __init__(self, frame: ttk.Frame) -> None:
        """Initialize the ImageFrame"""
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

        # Create canvas and bind it with scrollbars. Public for outer classes
        self.canvas = tk.Canvas(
            self._imframe, highlightthickness=0, xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set
        )
        self.canvas.grid(row=0, column=0, sticky="nswe")
        self.canvas.update()  # wait till canvas is created
        self.hbar.configure(command=self._scroll_x)  # bind scrollbars to the canvas
        self.vbar.configure(command=self._scroll_y)

        self.h_line1_id = self.canvas.create_line(0, 0, 0, 0, fill="white", width=1)  # horizontal line
        self.v_line1_id = self.canvas.create_line(0, 0, 0, 0, fill="white", width=1)  # vertical line

        self.h_line2_id = self.canvas.create_line(0, 0, 0, 0, fill="black", width=1)  # horizontal line
        self.v_line2_id = self.canvas.create_line(0, 0, 0, 0, fill="black", width=1)  # vertical line

        self.highlight_rectangle = HighlightRectangleState(self.canvas)

        self.annotation_ids: list[int] = []  # list of annotation object ids

    def set_image(self, image: Image.Image, annotation: YoloKeypointImage) -> None:
        self._image = image
        self._annotation = annotation
        self._pyramid: list[Image.Image] = [image]

        # Create image pyramid
        # Set ratio coefficient for image pyramid
        self._curr_img = 0  # current image from the pyramid
        self.imscale = 1.0  # scale for the canvas image zoom, public for outer classes
        self._scale = self.imscale
        self._reduction = 2  # reduction degree of image pyramid

        # Decide if this image huge or not
        self._huge_size = 14000  # define size of the huge image
        self.imwidth, self.imheight = self._image.size  # public for outer classes
        if self.imwidth * self.imheight > self._huge_size * self._huge_size:
            raise ValueError("Image is too big.")
        self._min_side = min(self.imwidth, self.imheight)  # get the smaller image side

        w, h = self._pyramid[-1].size
        while w > 512 and h > 512:  # top pyramid image is around 512 pixels in size
            w /= self._reduction  # divide on reduction degree
            h /= self._reduction  # divide on reduction degree
            self._pyramid.append(self._pyramid[-1].resize((int(w), int(h)), self._filter))

        if self.container is not None:
            self.canvas.delete(self.container)

        # Put image into container rectangle and use it to set proper coordinates to the image
        self.container = self.canvas.create_rectangle((0, 0, self.imwidth, self.imheight), width=0)

        self._image_set = True
        self.redraw_figures()  # method for child classes
        self._show_image()  # show image on the canvas
        self._draw_annotation(self._annotation)  # draw annotation on the canvas
        self.canvas.focus_set()  # set focus on the canvas

    def _reset_canvas(self) -> None:
        """Move canvas to the initial position"""
        self.set_image(self._image, self._annotation)

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
        self.canvas.configure(scrollregion=tuple(map(int, box_scroll)))  # set scroll region
        x1 = max(box_canvas[0] - box_image[0], 0)  # get coordinates (x1,y1,x2,y2) of the image tile
        y1 = max(box_canvas[1] - box_image[1], 0)
        x2 = min(box_canvas[2], box_image[2]) - box_image[0]
        y2 = min(box_canvas[3], box_image[3]) - box_image[1]
        if int(x2 - x1) > 0 and int(y2 - y1) > 0:  # show image if it's in the visible area
            image = self._pyramid[max(0, self._curr_img)].crop(  # crop current img from pyramid
                (int(x1 / self._scale), int(y1 / self._scale), int(x2 / self._scale), int(y2 / self._scale))
            )
            imagetk = ImageTk.PhotoImage(image.resize((int(x2 - x1), int(y2 - y1)), self._filter))
            imageid = self.canvas.create_image(
                max(box_canvas[0], box_img_int[0]), max(box_canvas[1], box_img_int[1]), anchor="nw", image=imagetk
            )
            self.canvas.lower(imageid)  # set image into background
            self.imagetk = imagetk  # keep an extra reference to prevent garbage-collection

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
        x_offset = self.canvas.canvasx(event.x)
        y_offset = self.canvas.canvasy(event.y)
        bbox = self.canvas.coords(self.container)
        self.canvas.coords(self.h_line1_id, bbox[0], y_offset, bbox[2], y_offset)  # horizontal line
        self.canvas.coords(self.v_line1_id, x_offset, bbox[1], x_offset, bbox[3])  # vertical line

        self.canvas.coords(self.h_line2_id, bbox[0], y_offset + 1, bbox[2], y_offset + 1)  # horizontal line
        self.canvas.coords(self.v_line2_id, x_offset + 1, bbox[1], x_offset + 1, bbox[3])  # vertical line

        self.highlight_rectangle.update_rectangle(event)

    def outside(self, x: int, y: int) -> bool:
        """Checks if the point (x,y) is outside the image area"""
        assert self.container is not None
        bbox = self.canvas.coords(self.container)  # get image area
        return not (bbox[0] < x < bbox[2] and bbox[1] < y < bbox[3])

    def wheel_callback(self, event: tk.Event) -> None:
        """Zoom with mouse wheel"""
        if not self._image_set:
            return
        x = self.canvas.canvasx(event.x)  # get coordinates of the event on the canvas
        y = self.canvas.canvasy(event.y)
        if self.outside(x, y):
            return  # zoom only inside image area
        scale = 1.0
        # Respond to Linux (event.num) or Windows (event.delta) wheel event
        if event.num == 5 or event.delta == -120:  # scroll down, smaller
            if round(self._min_side * self.imscale) < 30:
                return  # image is less than 30 pixels
            self.imscale /= self._delta
            scale /= self._delta
        if event.num == 4 or event.delta == 120:  # scroll up, bigger
            index = min(self.canvas.winfo_width(), self.canvas.winfo_height()) >> 1
            if index < self.imscale:
                return  # 1 pixel is bigger than the visible area
            self.imscale *= self._delta
            scale *= self._delta

        if scale != 1.0:
            self._set_position_and_scale(x, y, scale)  # set new scale

    def button1_click_callback(self, event: tk.Event) -> None:
        if not self._image_set:
            return
        if self.outside(event.x, event.y):
            return
        if not self.highlight_rectangle.is_drawing:
            self.highlight_rectangle.start_drawing(event)
        else:
            self.highlight_rectangle.finish_drawing()

    def _set_position_and_scale(self, x: int, y: int, scale: float) -> None:
        # Take appropriate image from the pyramid
        k = self.imscale  # temporary coefficient
        self._curr_img = min((-1) * int(math.log(k, self._reduction)), len(self._pyramid) - 1)
        self._scale = k * math.pow(self._reduction, max(0, self._curr_img))

        self.canvas.scale("all", x, y, scale, scale)  # rescale all objects
        self.highlight_rectangle.offset_position(x, y)  # move highlight rectangle
        # Redraw some figures before showing image on the screen
        self.redraw_figures()  # method for child classes
        self._show_image()

    def keystroke_callback(self, event: tk.Event) -> None:
        """Scrolling with the keyboard.
        Independent from the language of the keyboard, CapsLock, <Ctrl>+<key>, etc."""
        if not isinstance(event.state, int):
            return
        if event.state - self._previous_state == 4:  # means that the Control key is pressed
            return
        self._previous_state = event.state  # remember the last keystroke state
        # Up, Down, Left, Right keystrokes
        if event.keysym == "d":
            self._scroll_x("scroll", 1, "unit", event=event)
        elif event.keysym == "a":
            self._scroll_x("scroll", -1, "unit", event=event)
        elif event.keysym == "w":
            self._scroll_y("scroll", -1, "unit", event=event)
        elif event.keysym == "s":
            self._scroll_y("scroll", 1, "unit", event=event)
        elif event.keysym == "r":
            self._reset_canvas()
            self._show_image()

    def _draw_annotation(self, annotation: YoloKeypointImage) -> None:
        """Draw annotation on the canvas"""
        if not self._image_set:
            return
        assert self.container is not None
        for label in annotation.labels:
            self._draw_label(label)

    def _draw_label(self, label: YoloKeypointAnnotation) -> None:
        """Draw label on the canvas"""
        if not self._image_set:
            return
        assert self.container is not None
        bbox = self.canvas.coords(self.container)
        x0 = int(label.x0 * self.imwidth)
        y0 = int(label.y0 * self.imheight)
        x1 = int(label.x1 * self.imwidth)
        y1 = int(label.y1 * self.imheight)
        x0 = max(x0, bbox[0])
        y0 = max(y0, bbox[1])
        x1 = min(x1, bbox[2])
        y1 = min(y1, bbox[3])
        self.annotation_ids.append(self.canvas.create_rectangle(x0, y0, x1, y1, outline="red", width=2))
        for keypoint in label.keypoints:
            x = int(keypoint[0] * self.imwidth)
            y = int(keypoint[1] * self.imheight)
            x = max(x, bbox[0])
            y = max(y, bbox[1])
            x = min(x, bbox[2])
            y = min(y, bbox[3])
            self.annotation_ids.append(self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="red", outline="red"))
        self.annotation_ids.append(
            self.canvas.create_text(x0, y0 - 10, text=str(label.class_index), fill="red", font=("Arial", 12, "bold"))
        )

    def destroy(self) -> None:
        """ImageFrame destructor"""
        self._image.close()
        map(lambda i: i.close, self._pyramid)  # close all pyramid images
        del self._pyramid[:]  # delete pyramid list
        del self._pyramid  # delete pyramid variable
        self.canvas.destroy()
        self._imframe.destroy()
