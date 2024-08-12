from __future__ import annotations

import argparse
import uuid
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import cv2
import numpy as np
from perception_tools.config.model_metadata import LABEL_COLORS, ModelLabel


class ClickedLabelState(Enum):
    NOT_LABELING = auto()
    BOX_POINT = auto()
    BOX_DONE = auto()
    POSE_POINT = auto()


@dataclass
class LabeledRectangle:
    label_id: str
    model_label: ModelLabel
    point0: tuple[int, int]
    point1: tuple[int, int]
    pose_point0: tuple[int, int]
    pose_point1: tuple[int, int]
    enabled: bool = True


@dataclass
class ClickedRectangle:
    box_point0: tuple[int, int] | None = None
    box_point1: tuple[int, int] | None = None
    pose_point0: tuple[int, int] | None = None
    pose_point1: tuple[int, int] | None = None

    @classmethod
    def make_labeled(cls, other: ClickedRectangle, model_label: ModelLabel) -> LabeledRectangle:
        assert (
            other.box_point0 is not None
            and other.box_point1 is not None
            and other.pose_point0 is not None
            and other.pose_point1 is not None
        )
        return LabeledRectangle(
            str(uuid.uuid4()), model_label, other.box_point0, other.box_point1, other.pose_point0, other.pose_point1
        )


KEYMAP = {
    65361: "left",
    65363: "right",
    65362: "up",
    65364: "down",
}


class App:
    def __init__(self, image_paths: list[Path]) -> None:
        self.image_paths = image_paths
        self.image_index = 0
        self.active_image = np.zeros((512, 512, 3), np.uint8)
        self.clicked_rectangle = ClickedRectangle()
        self.clicked_state = ClickedLabelState.NOT_LABELING
        self.crosshair = (0, 0)
        self.active_labels: list[LabeledRectangle] = []
        self.labels: dict[Path, list[LabeledRectangle]] = {}
        self.mouse_events = {
            cv2.EVENT_LBUTTONDOWN: self.left_mouse_down_callback,
            cv2.EVENT_LBUTTONUP: self.left_mouse_up_callback,
            cv2.EVENT_MOUSEMOVE: self.mouse_move_callback,
        }
        self.keyboard_events = {
            "q": lambda: exit(),
            "r": self.reset_active_labels,
            "c": self.reset_label_state,
            "left": lambda: self.set_image(self.image_index - 1),
            "right": lambda: self.set_image(self.image_index + 1),
            "n": self.track_labels_to_next_image,
        }
        self.window_name = "Labeler"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, lambda event, x, y, flags, param: self.mouse_callback(event, x, y))
        for label in ModelLabel:
            cv2.createButton(
                label.name, lambda *args, x=label: self.set_active_label(ModelLabel(x)), None, cv2.QT_RADIOBOX, 0
            )
        self.active_label = ModelLabel.BACKGROUND

    def set_active_label(self, label: ModelLabel) -> None:
        print(f"Setting active label to {label}")
        self.active_label = label

    def reset_label_state(self) -> None:
        self.clicked_rectangle = ClickedRectangle()
        self.clicked_state = ClickedLabelState.NOT_LABELING

    def reset_active_labels(self) -> None:
        self.active_labels = []

    def create_label_entry(self, index: int, label: LabeledRectangle) -> None:
        cv2.createButton(
            f"{index}: {label.model_label.name}",
            lambda *args, x=label.label_id: self.set_label_entry_enable(args[0], x),
            None,
            cv2.QT_NEW_BUTTONBAR | cv2.QT_CHECKBOX,
            1,
        )

    def set_label_entry_enable(self, enabled: bool, uuid: str) -> None:
        for rectangle in self.active_labels:
            if rectangle.label_id == uuid:
                rectangle.enabled = enabled
                return

    def set_image(self, index: int) -> None:
        self.image_index = max(0, min(index, len(self.image_paths) - 1))
        print(f"Loading image {self.image_index + 1}/{len(self.image_paths)}")
        path = self.image_paths[self.image_index]
        if not path.is_file():
            raise FileNotFoundError(f"File not found: {path}")
        frame = cv2.imread(str(path))
        if frame is None:
            exit()
        self.active_image = frame
        self.labels[path] = self.active_labels
        self.reset_label_state()
        self.reset_active_labels()

    def track_labels_to_next_image(self) -> None:
        frame_gray_init = cv2.cvtColor(self.active_image, cv2.COLOR_BGR2GRAY)

        edges_list = []
        label_order = []
        for rectangle in self.active_labels:
            if not rectangle.enabled:
                continue
            edges_list.append(rectangle.point0)
            edges_list.append(rectangle.point1)
            edges_list.append((rectangle.point0[0], rectangle.point1[1]))
            edges_list.append((rectangle.point1[0], rectangle.point0[1]))
            edges_list.append(rectangle.pose_point0)
            edges_list.append(rectangle.pose_point1)
            label_order.append((rectangle.label_id, rectangle.model_label))
        edges = np.array(edges_list, dtype=np.float32).reshape(-1, 1, 2)

        self.set_image(self.image_index + 1)
        frame_gray = cv2.cvtColor(self.active_image, cv2.COLOR_BGR2GRAY)
        update_edges, status, errors = cv2.calcOpticalFlowPyrLK(
            frame_gray_init,
            frame_gray,
            edges,
            nextPts=None,
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        # only update edges if algorithm successfully tracked
        new_edges = update_edges[status == 1]

        for index in range(0, len(new_edges), 6):
            rectangle_corners = new_edges[index : index + 4]
            pose_point0 = tuple([int(x) for x in new_edges[index + 4]])
            pose_point1 = tuple([int(x) for x in new_edges[index + 5]])
            bounded_rect = [int(x) for x in cv2.boundingRect(rectangle_corners)]
            label_id, model_label = label_order[index // 6]
            rectangle = LabeledRectangle(
                label_id,
                model_label,
                (bounded_rect[0], bounded_rect[1]),
                (bounded_rect[0] + bounded_rect[2], bounded_rect[1] + bounded_rect[3]),
                pose_point0,
                pose_point1,
            )
            self.active_labels.append(rectangle)

    def update(self, frame: np.ndarray) -> None:
        if self.clicked_state == ClickedLabelState.BOX_POINT or self.clicked_state == ClickedLabelState.BOX_DONE:
            self.draw_clicked_box(frame)
        elif self.clicked_state == ClickedLabelState.POSE_POINT:
            self.draw_clicked_box(frame)
            self.draw_clicked_pose(frame)
        self.draw_crosshair(frame)
        for rectangle in self.active_labels:
            label = rectangle.model_label
            if not rectangle.enabled:
                continue
            if rectangle.point0 is not None and rectangle.point1 is not None:
                cv2.rectangle(frame, rectangle.point0, rectangle.point1, LABEL_COLORS[label].to_cv_color(), 1)
                cv2.arrowedLine(
                    frame, rectangle.pose_point0, rectangle.pose_point1, LABEL_COLORS[label].to_cv_color(), 2
                )

    def draw_clicked_box(self, frame: np.ndarray) -> None:
        active_color = self.get_active_color()
        assert self.clicked_rectangle.box_point0 is not None and self.clicked_rectangle.box_point1 is not None
        cv2.rectangle(frame, self.clicked_rectangle.box_point0, self.clicked_rectangle.box_point1, active_color, 1)

    def draw_clicked_pose(self, frame: np.ndarray) -> None:
        active_color = self.get_active_color()
        assert self.clicked_rectangle.pose_point0 is not None and self.clicked_rectangle.pose_point1 is not None
        cv2.arrowedLine(frame, self.clicked_rectangle.pose_point0, self.clicked_rectangle.pose_point1, active_color, 2)

    def get_active_color(self) -> tuple[int, int, int]:
        return LABEL_COLORS[self.active_label].to_cv_color()

    def draw_crosshair(self, frame: np.ndarray) -> None:
        active_color = self.get_active_color()
        cv2.line(frame, (self.crosshair[0], 0), (self.crosshair[0], frame.shape[0]), active_color, 1)
        cv2.line(frame, (0, self.crosshair[1]), (frame.shape[1], self.crosshair[1]), active_color, 1)
        cv2.putText(
            frame,
            self.active_label.name,
            (self.crosshair[0], self.crosshair[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            active_color,
        )

    def run(self) -> None:
        try:
            self.set_image(0)
            while True:
                frame = self.active_image.copy()
                self.update(frame)
                cv2.imshow(self.window_name, frame)
                raw_key = cv2.waitKeyEx(1)
                if raw_key == -1:
                    continue
                if raw_key <= 0xFF:
                    key = chr(raw_key & 0xFF)
                else:
                    key = KEYMAP.get(raw_key, None)
                print(f"Key pressed: {key}")
                if key in self.keyboard_events:
                    self.keyboard_events[key]()
        finally:
            cv2.destroyAllWindows()

    def mouse_callback(self, event: int, x: int, y: int) -> None:
        self.mouse_events[event](x, y)
        self.crosshair = (x, y)

    def left_mouse_down_callback(self, x: int, y: int) -> None:
        if self.clicked_state == ClickedLabelState.NOT_LABELING:
            self.clicked_rectangle.box_point0 = (x, y)
            self.clicked_rectangle.box_point1 = (x, y)
            self.clicked_state = ClickedLabelState.BOX_POINT
        elif self.clicked_state == ClickedLabelState.BOX_DONE:
            self.clicked_rectangle.pose_point0 = (x, y)
            self.clicked_rectangle.pose_point1 = (x, y)
            self.clicked_state = ClickedLabelState.POSE_POINT
        else:
            raise ValueError(f"Invalid state {self.clicked_state}")

    def left_mouse_up_callback(self, x: int, y: int) -> None:
        if self.clicked_state == ClickedLabelState.NOT_LABELING:
            return
        elif self.clicked_state == ClickedLabelState.BOX_POINT:
            self.clicked_state = ClickedLabelState.BOX_DONE
        elif self.clicked_state == ClickedLabelState.POSE_POINT:
            labeled = ClickedRectangle.make_labeled(self.clicked_rectangle, self.active_label)
            self.create_label_entry(len(self.active_labels), labeled)
            self.active_labels.append(labeled)
            self.clicked_rectangle = ClickedRectangle()
            self.clicked_state = ClickedLabelState.NOT_LABELING
        else:
            raise ValueError(f"Invalid state {self.clicked_state}")

    def mouse_move_callback(self, x: int, y: int) -> None:
        if self.clicked_state == ClickedLabelState.BOX_POINT:
            self.clicked_rectangle.box_point1 = (x, y)
        elif self.clicked_state == ClickedLabelState.POSE_POINT:
            self.clicked_rectangle.pose_point1 = (x, y)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, nargs="+", help="path to input video file(s)")
    args = parser.parse_args()
    images = [Path(image) for image in args.images]
    App(images).run()


if __name__ == "__main__":
    main()
