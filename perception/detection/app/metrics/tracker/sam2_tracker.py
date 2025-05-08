from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from app.config.metrics_tool.tracker.sam2_tracker_config import Sam2TrackerConfig
from app.metrics.list_cache_image_paths import list_cache_image_paths
from app.metrics.tracker.tracker import TrackedPoints, TrackerInterface
from app.metrics.tracker.tracking_data import TrackingData
from bw_shared.messages.header import Header
from sam2.build_sam import build_sam2_video_predictor


class Sam2Tracker(TrackerInterface):
    def __init__(self, config: Sam2TrackerConfig):
        self.config = config
        self.device: torch.device | None = None
        self.predictor: torch.nn.Module | None = None
        self.is_initialized = False
        self.interference_state: dict = {}
        self.tracking_data: dict[int, TrackingData] = {}
        self.images_dir: Path | None = None
        self.tracked_points: TrackedPoints = {}

    def initialize(self, images_dir: Path) -> None:
        self._first_time_init()
        if not self.is_initialized:
            raise RuntimeError("Tracker is not initialized")
        assert self.predictor is not None
        self.images_dir = images_dir
        self.inference_state = self.predictor.init_state(video_path=str(images_dir))  # type: ignore

    def add_track_points(self, frame_num: int, object_id: int, points: list[tuple[int, int]]) -> None:
        self._add_points(frame_num, object_id, points, 1)

    def add_reject_points(self, frame_num: int, object_id: int, points: list[tuple[int, int]]) -> None:
        self._add_points(frame_num, object_id, points, 0)

    def _add_points(self, frame_num: int, object_id: int, points: list[tuple[int, int]], label: int) -> None:
        if not self.is_initialized:
            raise RuntimeError("Tracker is not initialized")
        assert self.predictor is not None
        key = (frame_num, label)
        self.tracked_points.setdefault(key, {}).setdefault(object_id, []).extend(points)
        points_array = np.array(points, dtype=np.float32)

        # for labels, `1` means positive click and `0` means negative click
        labels = np.array([label for _ in range(len(points))], np.int32)
        _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(  # type: ignore
            inference_state=self.inference_state,
            frame_idx=frame_num,
            obj_id=object_id,
            points=points_array,
            labels=labels,
        )
        self._set_tracking_data(frame_num, out_obj_ids, out_mask_logits)

    def compute(self) -> None:
        if not self.is_initialized:
            raise RuntimeError("Tracker is not initialized")
        assert self.predictor is not None

        for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(self.inference_state):  # type: ignore
            self._set_tracking_data(out_frame_idx, out_obj_ids, out_mask_logits)
        for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(  # type: ignore
            self.inference_state, reverse=True
        ):
            self._set_tracking_data(out_frame_idx, out_obj_ids, out_mask_logits)

    def _set_tracking_data(self, out_frame_idx: int, out_obj_ids: list[int], out_mask_logits: Any) -> None:
        frame_contours = {}
        header = self._index_to_header(out_frame_idx)
        for index, out_obj_id in enumerate(out_obj_ids):
            mask = (out_mask_logits[index] > 0.0).cpu().numpy().astype(np.uint8)
            height, width = mask.shape[-2:]
            mask_image = mask.reshape(height, width, 1) * 255
            contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            frame_contours[out_obj_id] = contours
            if hierarchy is None:  # empty mask
                print(f"Empty mask for object {out_obj_id}")
        self.tracking_data[out_frame_idx] = TrackingData(header=header, contours=frame_contours)

    def get_tracking(self, frame_num: int) -> TrackingData | None:
        return self.tracking_data.get(frame_num)

    def set_tracking(self, frame_num: int, tracking_data: TrackingData) -> None:
        self.tracking_data[frame_num] = tracking_data

    def get_tracked_points(self) -> TrackedPoints:
        return self.tracked_points

    def reset(self) -> None:
        if not self.is_initialized:
            raise RuntimeError("Tracker is not initialized")
        assert self.predictor is not None

        self.predictor.reset_state(self.inference_state)  # type: ignore
        self.tracking_data = {}
        self.tracked_points = {}

    def _first_time_init(self) -> None:
        if self.is_initialized:
            return
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        self.device = torch.device("cuda")

        if self.device.type == "cuda":
            # use bfloat16 for the entire notebook
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True

        self.predictor = build_sam2_video_predictor(self.config.model_cfg, self.config.checkpoint, self.device)
        self.is_initialized = True

    def _index_to_header(self, index: int) -> Header:
        if self.images_dir is None:
            raise RuntimeError("Images directory is not set")

        paths = list_cache_image_paths(self.images_dir)
        return Header(stamp=float(paths[index].stem) / 1000.0, frame_id="", seq=index)
