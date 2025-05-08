import logging
import tkinter as tk
from tkinter import ttk
from typing import Callable

from bw_shared.enums.label import ModelLabel


class LabelSelectionToolbar:
    def __init__(
        self, parent_frame: ttk.Frame, labels: list[ModelLabel], on_label_selected: Callable[[int], None]
    ) -> None:
        self.parent_frame = parent_frame
        self.labels = labels
        self.on_label_selected = on_label_selected
        self.selected_label: ModelLabel | None = None
        self.label_values = [label.value for label in labels]

        self.radio_var = tk.IntVar(self.parent_frame, 0)
        self.radio_buttons: list[ttk.Radiobutton] = []
        for index, label in enumerate(self.labels):
            radio_button = ttk.Radiobutton(
                self.parent_frame,
                text=label.value,
                variable=self.radio_var,
                value=index,
                command=lambda index=index: self.on_label_selected(index),  # type: ignore
            )
            radio_button.pack(side=tk.LEFT, padx=5)
            self.radio_buttons.append(radio_button)

        self.logger = logging.getLogger(self.__class__.__name__)

    def set_label(self, index: int) -> None:
        if not (0 <= index < len(self.label_values)):
            self.logger.warning(f"Index {index} is out of range for label values.")
            return
        self.radio_var.set(index)
        self.selected_label = ModelLabel(self.label_values[index])
        self.logger.debug(f"Selecting {self.selected_label} at index {index}.")
        self.on_label_selected(index)

    def on_label_change(self, event: tk.Event) -> None:
        selected_index = self.radio_var.get()
        if selected_index < 0 or selected_index >= len(self.label_values):
            self.logger.warning(f"Selected index {selected_index} is out of range.")
            return
        selected_label = self.label_values[selected_index]
        self.selected_label = ModelLabel(selected_label)
        self.on_label_selected(selected_index)
