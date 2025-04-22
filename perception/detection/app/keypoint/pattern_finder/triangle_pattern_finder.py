import cv2
import numpy as np
from app.config.keypoint_config.pattern_finder_config.triangle_pattern_finder_config import TrianglePatternFinderConfig
from app.keypoint.pattern_finder.pattern_finder import PatternFinder
from bw_interfaces.msg import UVKeypoint
from bw_shared.geometry.xy import XY


class TrianglePatternFinder(PatternFinder):
    def __init__(self, config: TrianglePatternFinderConfig) -> None:
        self.config = config

    def find(
        self, image: np.ndarray, contour: np.ndarray, debug_image: np.ndarray | None = None
    ) -> tuple[UVKeypoint | None, UVKeypoint | None]:
        mask = np.zeros(image.shape[0:2], dtype=np.uint8)
        mask = cv2.drawContours(mask, [contour], -1, (255,), -1)
        crop = cv2.boundingRect(contour)
        cropped_image = image[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
        cropped_mask = mask[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
        eroded_mask = cv2.erode(cropped_mask, np.ones((3, 3), np.uint8), iterations=1)
        masked_image = cv2.bitwise_and(cropped_image, cropped_image, mask=eroded_mask)
        gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        cv2.normalize(gray, gray, 0, 255, cv2.NORM_MINMAX, mask=eroded_mask)
        thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        filtered_contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(filtered_contours) == 0:
            return None, None
        largest_contour = max(filtered_contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP(largest_contour, 0.07 * cv2.arcLength(largest_contour, True), True)
        if len(approx) != 3:
            return None, None

        if debug_image is not None:
            approx_absolute = approx + np.array([crop[0], crop[1]])
            cv2.drawContours(debug_image, [approx_absolute], -1, (0, 255, 0), 3)  # type: ignore

        approx_reduced = approx[:, 0]
        lengths = [np.linalg.norm(approx_reduced[(i + 1) % 3] - approx_reduced[i]) for i in range(3)]
        shortest_index = lengths.index(min(lengths))
        shortest_line = approx_reduced[shortest_index], approx_reduced[(shortest_index + 1) % 3]
        shortest_length_angle = np.arctan2(
            shortest_line[1][1] - shortest_line[0][1],
            shortest_line[1][0] - shortest_line[0][0],
        )
        back_xy = XY(
            (shortest_line[0][0] + shortest_line[1][0]) / 2,
            (shortest_line[0][1] + shortest_line[1][1]) / 2,
        )
        front_xys: list[XY] = []
        angles = []
        for index in range(len(lengths)):
            front_xy = XY(
                (approx_reduced[(index) % 3][0]),
                (approx_reduced[(index) % 3][1]),
            )
            front_xys.append(front_xy)
            angle = np.arctan2(
                back_xy.y - front_xy.y,
                back_xy.x - front_xy.x,
            )
            delta_angle = abs(shortest_length_angle - angle) % np.pi
            angles.append(delta_angle)

        max_angle_index = np.argmax(angles)
        selected_front_xy = front_xys[max_angle_index]

        front_keypoint = UVKeypoint(x=selected_front_xy.x + crop[0], y=selected_front_xy.y + crop[1])
        back_keypoint = UVKeypoint(x=back_xy.x + crop[0], y=back_xy.y + crop[1])

        if debug_image is not None:
            cv2.line(
                debug_image,
                (int(front_keypoint.x), int(front_keypoint.y)),
                (int(back_keypoint.x), int(back_xy.y)),
                (0, 0, 255),
                2,
            )

        return front_keypoint, back_keypoint
