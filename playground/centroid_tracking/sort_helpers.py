import time
from typing import Optional

import numpy as np
from filterpy.kalman import KalmanFilter
from numpy import typing as npt
from scipy.optimize import linear_sum_assignment

BoundingBoxXYXYArray = npt.NDArray[np.floating]  # [[x1, y1, x2, y2], ...]
BoundingBoxXYXY = npt.NDArray[np.floating]  # [x1, y1, x2, y2]
BoundingBoxState = npt.NDArray[np.floating]  # [[x], [y], [s], [r]]
TrackedState = npt.NDArray[np.floating]  # [[x], [y], [s], [r], [vx], [vy], [vs]]


def linear_assignment(cost_matrix: npt.NDArray[np.floating]) -> npt.NDArray[np.floating]:
    return np.vstack(linear_sum_assignment(cost_matrix)).T


def iou_batch(bb_test: BoundingBoxXYXYArray, bb_gt: BoundingBoxXYXYArray) -> npt.NDArray[np.floating]:
    """
    Compute the Intersection-Over-Union of a batch of boxes in the form [x1,y1,x2,y2].
    Args:
        bb_test: (np.ndarray) bounding boxes, sized [N, 4].
        bb_gt: (np.ndarray) bounding boxes, sized [M, 4].
    Returns:
        (np.ndarray) iou, sized [N, M].
    """
    bb_gt = np.expand_dims(bb_gt, axis=0)  # [1, M, 4]
    bb_test = np.expand_dims(bb_test, axis=1)  # [N, 1, 4]

    xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
    yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
    xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
    yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
    w = np.maximum(0.0, xx2 - xx1)
    h = np.maximum(0.0, yy2 - yy1)
    wh = w * h
    o = wh / (
        (bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])
        + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1])
        - wh
    )
    return o


def convert_bbox_to_z(bbox: BoundingBoxXYXY) -> BoundingBoxState:
    """
    Takes a bounding box in the form [x1, y1, x2, y2] and returns z in the form
    [x, y, s, r] where x, y is the center of the box and s is the scale/area and r is
    the aspect ratio
    """
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = bbox[0] + w / 2.0
    y = bbox[1] + h / 2.0
    s = w * h  # scale is just area
    r = w / float(h)
    return np.array([x, y, s, r]).reshape((4, 1))


def convert_x_to_bbox(x: BoundingBoxState) -> BoundingBoxXYXY:
    """
    Takes a bounding box in the center form [x,y,s,r] and returns it in the form
      [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
    """
    x_flat = x.flatten()
    w = np.sqrt(x_flat[2] * x_flat[3])
    h = x_flat[2] / w
    return np.array(
        [
            x_flat[0] - w / 2.0,
            x_flat[1] - h / 2.0,
            x_flat[0] + w / 2.0,
            x_flat[1] + h / 2.0,
        ]
    )


class BoxTracker:
    id_count = 0

    def __init__(self, bbox: BoundingBoxXYXY, min_hits: int, max_age: int, tracker_index: Optional[int] = None) -> None:
        """
        Initializes a tracker using initial bounding box.
        """
        self.min_hits = min_hits  # number of consecutive detections before track is confirmed
        self.max_age = max_age  # number of consecutive misses before track is deleted
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.array(
            [
                [1, 0, 0, 0, 1, 0, 0],
                [0, 1, 0, 0, 0, 1, 0],
                [0, 0, 1, 0, 0, 0, 1],
                [0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 1],
            ]
        )
        self.kf.H = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0],
            ]
        )

        self.kf.R[2:, 2:] *= 10.0  # measurement uncertainty
        self.kf.P[4:, 4:] *= 1000.0  # give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10.0  # covariance matrix
        self.kf.Q[-1, -1] *= 0.01  # process uncertainty
        self.kf.Q[4:, 4:] *= 0.01

        self.kf.x[:4] = convert_bbox_to_z(bbox)  # initial state

        self.ticks_since_update = 0
        if tracker_index is None:
            self.id = BoxTracker.id_count
            BoxTracker.id_count += 1
        else:
            self.id = tracker_index

        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def is_confirmed(self, frame_count: int) -> bool:
        """
        Returns True if the track is confirmed.

        frame_count is the current frame count since the last reset.
        """
        return self.ticks_since_update < 1 and (self.hit_streak >= self.min_hits or frame_count < self.min_hits)

    def is_stale(self) -> bool:
        return self.age > self.max_age

    def update(self, bbox: BoundingBoxXYXY) -> None:
        """
        Updates the tracker with a new bounding box.
        """
        self.ticks_since_update = 0
        self.hits += 1
        self.hit_streak += 1

        self.kf.update(convert_bbox_to_z(bbox))

    def predict(self) -> None:
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        if self.kf.x[6] + self.kf.x[2] <= 0:
            self.kf.x[6] = 0.0
        self.kf.predict()
        self.age += 1
        if self.ticks_since_update > 0:
            self.hit_streak = 0
        self.ticks_since_update += 1

    def get_state(self) -> BoundingBoxXYXY:
        """
        Returns the current bounding box estimate.
        """
        return convert_x_to_bbox(self.kf.x)


def associate_detections_to_trackers(
    detections: BoundingBoxXYXYArray, trackers: BoundingBoxXYXYArray, iou_threshold: float = 0.3
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if len(trackers) == 0:
        return [], list(range(len(detections))), []

    iou_matrix = iou_batch(detections, trackers)

    if min(iou_matrix.shape) > 0:
        a = (iou_matrix > iou_threshold).astype(np.int32)
        if a.sum(axis=1).max() == 1 and a.sum(axis=0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(-iou_matrix)
    else:
        matched_indices = np.empty(shape=(0, 2))

    unmatched_detections = []
    for detect_index in range(len(detections)):
        if detect_index not in matched_indices[:, 0]:
            unmatched_detections.append(detect_index)
    unmatched_trackers = []
    for track_index in range(len(trackers)):
        if track_index not in matched_indices[:, 1]:
            unmatched_trackers.append(track_index)

    # filter out matched with low IOU
    matches = []
    for detect_index, track_index in matched_indices:
        if iou_matrix[detect_index, track_index] < iou_threshold:
            unmatched_detections.append(detect_index)
            unmatched_trackers.append(track_index)
        else:
            matches.append((detect_index, track_index))
    if len(matches) == 0:
        matches = []

    return matches, unmatched_detections, unmatched_trackers
