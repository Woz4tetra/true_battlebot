import numpy as np
from app.config.keypoint_config.pattern_finder_config.noop_pattern_finder_config import NoopPatternFinderConfig
from app.keypoint.pattern_finder.pattern_finder import PatternFinder
from bw_interfaces.msg import UVKeypoint


class NoopPatternFinder(PatternFinder):
    def __init__(self, config: NoopPatternFinderConfig) -> None:
        self.config = config

    def find(
        self, image: np.ndarray, contour: np.ndarray, debug_image: np.ndarray | None = None
    ) -> tuple[UVKeypoint | None, UVKeypoint | None]:
        return None, None
