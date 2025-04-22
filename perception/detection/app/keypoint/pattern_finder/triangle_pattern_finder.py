import logging

import cv2
import numpy as np
from app.config.keypoint_config.pattern_finder_config.triangle_pattern_finder_config import TrianglePatternFinderConfig
from app.keypoint.pattern_finder.pattern_finder import PatternFinder
from bw_interfaces.msg import UVKeypoint
from bw_shared.geometry.xy import XY
from perception_tools.messages.image import Image


class TrianglePatternFinder(PatternFinder):
    def __init__(self, config: TrianglePatternFinderConfig) -> None:
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)

    def find(
        self, gray_image: np.ndarray, contour: np.ndarray, debug_image: Image | None = None
    ) -> tuple[UVKeypoint | None, UVKeypoint | None]:
        mask = np.zeros(gray_image.shape[0:2], dtype=np.uint8)
        crop = cv2.boundingRect(contour)
        mask = cv2.drawContours(mask, contour.astype(np.int32), -1, (255,), -1)  # type: ignore
        cropped_image = gray_image[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
        cropped_mask = mask[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
        if cropped_image.size == 0:
            self.logger.debug("Cropped image is empty")
            return None, None
        normalized = cv2.normalize(cropped_image, None, 0, 255, cv2.NORM_MINMAX, mask=cv2.bitwise_not(cropped_mask))  # type: ignore
        thresholded = cv2.threshold(normalized, self.config.threshold, 255, cv2.THRESH_BINARY)[1]
        filtered_contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(filtered_contours) == 0:
            self.logger.debug("No contours found")
            return None, None
        largest_contour = max(filtered_contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP(
            largest_contour, self.config.contour_smoothing * cv2.arcLength(largest_contour, True), True
        )
        num_points = 3
        if len(approx) != num_points:
            self.logger.debug(f"Contour is not a triangle: {len(approx)}")
            return None, None

        if debug_image is not None:
            approx_absolute = approx + np.array([crop[0], crop[1]])
            cv2.drawContours(debug_image.data, [approx_absolute], -1, (0, 255, 0), 3)  # type: ignore

        approx_reduced = approx[:, 0]
        lengths = [np.linalg.norm(approx_reduced[(i + 1) % num_points] - approx_reduced[i]) for i in range(num_points)]
        shortest_index = lengths.index(max(lengths))
        longest_line = approx_reduced[shortest_index], approx_reduced[(shortest_index + 1) % num_points]
        longest_length_angle = np.arctan2(
            longest_line[1][1] - longest_line[0][1],
            longest_line[1][0] - longest_line[0][0],
        )
        back_xy = XY(
            (longest_line[0][0] + longest_line[1][0]) / 2,
            (longest_line[0][1] + longest_line[1][1]) / 2,
        )
        front_xys: list[XY] = []
        angles = []
        for index in range(num_points):
            if index == shortest_index:
                continue
            front_xy = XY(
                (approx_reduced[index % num_points][0]),
                (approx_reduced[index % num_points][1]),
            )
            front_xys.append(front_xy)
            angle = np.arctan2(
                back_xy.y - front_xy.y,
                back_xy.x - front_xy.x,
            )
            delta_angle = abs(longest_length_angle - angle) % np.pi
            angles.append(delta_angle)

        max_angle_index = np.argmax(angles)
        selected_front_xy = front_xys[max_angle_index]

        front_keypoint = UVKeypoint(x=selected_front_xy.x + crop[0], y=selected_front_xy.y + crop[1])
        back_keypoint = UVKeypoint(x=back_xy.x + crop[0], y=back_xy.y + crop[1])

        if debug_image is not None:
            cv2.line(
                debug_image.data,
                (int(front_keypoint.x), int(front_keypoint.y)),
                (int(back_keypoint.x), int(back_keypoint.y)),
                (0, 0, 255),
                3,
            )

        return front_keypoint, back_keypoint
