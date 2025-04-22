from abc import ABC, abstractmethod

import numpy as np
from bw_interfaces.msg import UVKeypoint
from perception_tools.messages.image import Image


class PatternFinder(ABC):
    @abstractmethod
    def find(
        self, gray_image: np.ndarray, contour: np.ndarray, debug_image: Image | None = None
    ) -> tuple[UVKeypoint | None, UVKeypoint | None]:
        """
        Find the pattern in the image using the contours.

        Args:
            gray_image (np.ndarray): The input image. It should be a grayscale image.
            contour (np.ndarray): The contour to find the pattern within.
            debug_image (Image | None): Optional debug image for visualization.
                If provided, the function may draw the found keypoints on this image for debugging purposes.

        Returns:
            tuple[UVKeypoint, UVKeypoint]: Identified front and back keypoints in pixel coordinates.
                If the pattern is not found, returns (None, None). If one of the keypoints is not found,
                returns (None, UVKeypoint) or (UVKeypoint, None).
        """
        ...
