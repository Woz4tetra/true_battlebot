from dataclasses import dataclass


@dataclass
class VideoFilterConfig:
    dilate_kernel_size: int = 3
    dilate_iterations: int = 3
    contour_min_size: int = 1000
    min_time_interval: float = 0.0
    enable_motion_detection: bool = False
    max_frames: int = 500
