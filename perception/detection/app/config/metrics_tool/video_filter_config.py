from dataclasses import dataclass


@dataclass
class VideoFilterConfig:
    # initial filtering
    min_time_interval: float = 0.0
    max_frames: int = 500

    # background isolation
    median_window_size: int = 100

    # blob detection
    blobs_mask_threshold: int = 100
    blob_min_size: int = 1000
    max_number_of_blobs: int = 1000
    num_points_per_blob: int = 8
