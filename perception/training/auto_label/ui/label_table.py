import logging
from dataclasses import dataclass
from tkinter import ttk
from typing import Callable

from perception_tools.training.keypoints_config import KeypointsConfig
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage


@dataclass
class LabelTableRow:
    row_index: int
    label: ttk.Label
    delete_button: ttk.Button
    is_visible: bool = False

    def show(self) -> None:
        if self.is_visible:
            return
        self.label.grid(row=self.row_index + 1, column=0, sticky="nsew", padx=5, pady=5)
        self.delete_button.grid(row=self.row_index + 1, column=1, sticky="nsew", padx=5, pady=5)
        self.is_visible = True

    def hide(self) -> None:
        if not self.is_visible:
            return
        self.label.grid_forget()
        self.delete_button.grid_forget()
        self.is_visible = False


class LabelTable:
    def __init__(
        self,
        parent_frame: ttk.Frame,
        keypoints_config: KeypointsConfig,
        delete_callback: Callable[[int], YoloKeypointImage | None],
        highlight_callback: Callable[[int], None],
        unhighlight_callback: Callable[[int], None],
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.keypoints_config = keypoints_config
        self.label_table_frame = parent_frame
        self.rows: list[LabelTableRow] = []
        self.delete_callback = delete_callback
        self.highlight_callback = highlight_callback
        self.unhighlight_callback = unhighlight_callback
        self.placeholder_label1 = ttk.Label(self.label_table_frame, text="", width=12)
        self.placeholder_label2 = ttk.Label(self.label_table_frame, text="", width=12)
        self.placeholder_label1.grid(row=0, column=0, sticky="nsew")
        self.placeholder_label2.grid(row=0, column=1, sticky="nsew")

    def _add_label(self, row_index: int, annotation: YoloKeypointAnnotation) -> LabelTableRow:
        label_name = self.keypoints_config.labels[annotation.class_index].value
        label = ttk.Label(self.label_table_frame, text=label_name)
        delete_button = ttk.Button(
            self.label_table_frame,
            text="Delete",
        )
        row = LabelTableRow(row_index, label, delete_button)
        delete_button.config(command=lambda row=row: self.delete_annotation(row))
        delete_button.bind("<Enter>", lambda event, row=row: self.highlight_callback(row.row_index))
        delete_button.bind("<Leave>", lambda event, row=row: self.unhighlight_callback(row.row_index))
        self.logger.debug(f"Adding label: {label_name} at index {row_index}")
        return row

    def _get_label_name(self, annotation: YoloKeypointAnnotation) -> str:
        return self.keypoints_config.labels[annotation.class_index].value

    def populate_table(self, annotation: YoloKeypointImage) -> None:
        labels = annotation.labels
        # hide extra rows
        for row in self.rows:
            row.hide()

        # add new rows
        if len(self.rows) < len(labels):
            for index in range(len(self.rows), len(labels)):
                row = self._add_label(index, labels[index])
                self.rows.append(row)

        # update existing rows
        for index, row in enumerate(self.rows):
            if index < len(labels):
                row.label.config(text=self._get_label_name(labels[index]))
                row.show()

    def hide_table(self) -> None:
        for row in self.rows:
            row.hide()
        self.current_annotation = None
        self.logger.debug("Cleared label table.")

    def delete_annotation(self, row: LabelTableRow) -> None:
        if new_annotation := self.delete_callback(row.row_index):
            self.populate_table(new_annotation)
        else:
            self.hide_table()
