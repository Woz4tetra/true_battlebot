import numpy as np
from sort_helpers import (
    BoundingBoxXYXY,
    BoundingBoxXYXYArray,
    BoxTracker,
    associate_detections_to_trackers,
)


class Sorter:
    def __init__(self, num_trackers: int = 0, max_age: int = 1, min_hits: int = 3, iou_threshold: float = 0.3) -> None:
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold

        self.trackers: list[BoxTracker] = []
        self.frame_count = 0
        self.num_trackers = num_trackers

    def reset(self) -> None:
        self.trackers = []
        self.frame_count = 0

    def update(self, detections: BoundingBoxXYXYArray) -> dict[int, BoundingBoxXYXY]:
        if detections.size == 0:
            detections = np.empty((0, 4))
        self.frame_count += 1

        # get predicted locations from existing trackers.
        tracked_bboxes = []
        delete_indices = []
        for tracker_index, tracker in enumerate(self.trackers):
            tracker.predict()
            bbox_state = tracker.get_state()
            tracked_bboxes.append(bbox_state)
            if np.any(np.isnan(bbox_state)):
                delete_indices.append(tracker_index)
        for tracker_index in reversed(delete_indices):
            del self.trackers[tracker_index]
            del tracked_bboxes[tracker_index]

        matches, unmatched_detections, unmatched_trackers = associate_detections_to_trackers(
            detections, np.array(tracked_bboxes), self.iou_threshold
        )
        for detect_index, track_index in matches:
            tracker = self.trackers[track_index]
            tracker.update(detections[detect_index])

        for detect_index in unmatched_detections:
            if self.num_trackers > 0 and len(self.trackers) >= self.num_trackers:
                break
            tracker = BoxTracker(
                detections[detect_index],
                self.min_hits,
                self.max_age,
                tracker_index=len(self.trackers) + 1 if self.num_trackers > 0 else None,
            )
            self.trackers.append(tracker)

        result = {}
        for tracker in reversed(self.trackers):
            # if not tracker.is_confirmed(self.frame_count):
            #     continue
            if tracker.is_stale():
                self.trackers.remove(tracker)
                continue
            bbox = tracker.get_state()
            result[tracker.id] = bbox

        return result
