from abc import ABC, abstractmethod
from pathlib import Path

from app.metrics.tracker.tracking_data import TrackingData

TrackedPoints = dict[tuple[int, int], dict[int, list[tuple[int, int]]]]


class TrackerInterface(ABC):
    @abstractmethod
    def initialize(self, images_dir: Path) -> None: ...

    @abstractmethod
    def add_track_points(self, frame_num: int, object_id: int, points: list[tuple[int, int]]) -> None: ...

    @abstractmethod
    def add_reject_points(self, frame_num: int, object_id: int, points: list[tuple[int, int]]) -> None: ...

    @abstractmethod
    def compute(self) -> None: ...

    @abstractmethod
    def get_tracking(self, frame_num: int) -> TrackingData | None: ...

    @abstractmethod
    def set_tracking(self, frame_num: int, tracking_data: TrackingData) -> None: ...

    @abstractmethod
    def get_tracked_points(self) -> TrackedPoints: ...

    @abstractmethod
    def reset(self) -> None: ...
