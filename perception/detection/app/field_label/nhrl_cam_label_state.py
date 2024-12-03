import json

import numpy as np
from app.field_label.click_state import ClickState


class NhrlCamLabelState:
    def __init__(self) -> None:
        self.unhighlighted_radius = 3
        self.highlighted_radius = 10
        self.highlighted_index = -1
        self.image_points = np.zeros((0, 2), dtype=np.int32)
        self.did_click = False
        self.click_state = ClickState.UP

    def save_label_state(self, save_path: str) -> None:
        state = {
            "image_points": self.image_points.tolist(),
        }
        with open(save_path, "w") as file:
            json.dump(state, file)

    def load_label_state(self, load_path: str) -> None:
        with open(load_path, "r") as file:
            state = json.load(file)
            self.image_points = np.array(state["image_points"], dtype=np.int32)

    def update_highlighted_index(self, mouse_point: tuple[int, int]) -> None:
        mouse_array = np.array(mouse_point)
        for index, point in enumerate(self.image_points):
            if np.linalg.norm(point - mouse_array) < self.highlighted_radius:
                self.highlighted_index = index
                return
        self.highlighted_index = -1
