from app.config.metrics_tool.tracker.sam2_tracker_config import Sam2TrackerConfig
from app.config.metrics_tool.tracker.tracker_types import TrackerConfig
from app.container import Container
from app.metrics.tracker.tracker import TrackerInterface

from .sam2_tracker import Sam2Tracker


def load_tracker(container: Container, config: TrackerConfig) -> TrackerInterface:
    if isinstance(config, Sam2TrackerConfig):
        return Sam2Tracker(config)
    else:
        raise ValueError(f"Unknown segmentation config type: {type(config)}")
