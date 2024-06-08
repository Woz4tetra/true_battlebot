import cv2
import numpy as np
from bw_shared.geometry.in_plane import dist_from_point_to_line, line_line_intersection
from detectron2.data.transforms.augmentation import Augmentation, Transform


class HomographyTransform(Transform):
    def __init__(self, src_corners, dst_corners, width: int, height: int, expand=True):
        super().__init__()

        self.expand = expand
        self.width = width
        self.height = height
        self.tf_image, self.scale_factor = self.perspective_matrix(src_corners, dst_corners)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        self = cls.__new__(cls)
        self.tf_image = matrix
        return self

    def apply_image(self, img):
        """
        img should be a numpy array, formatted as Height * Width * Nchannels
        """
        width = img.shape[1]
        height = img.shape[0]
        warped = cv2.warpPerspective(img, self.tf_image, (width, height))
        if not self.expand:
            return warped
        width_delta = abs(int((self.scale_factor * width - width) / 2))
        height_delta = abs(int((self.scale_factor * height - height) / 2))
        if self.scale_factor > 1:
            expanded = cv2.copyMakeBorder(
                warped, height_delta, height_delta, width_delta, width_delta, cv2.BORDER_CONSTANT
            )
        else:
            expanded = warped[height_delta : height - height_delta, width_delta : width - width_delta]
        scaled = cv2.resize(expanded, (width, height))
        return scaled

    def apply_coords(self, coords):
        """
        coords should be a N * 2 array-like, containing N couples of (x, y) points
        """
        coords = np.asarray(coords, dtype=float)
        applied_coords = np.concatenate([coords, np.ones((coords.shape[0], 1))], axis=1)
        applied_coords = np.tensordot(applied_coords, self.scale_factor, axes=0)[:, :2]
        applied_coords = cv2.transform(coords[:, np.newaxis, :], self.tf_image)[:, 0, :2]
        return applied_coords

    def perspective_matrix(self, src_corners, dst_corners) -> tuple[np.ndarray, float]:
        mat = cv2.getPerspectiveTransform(src_corners, dst_corners)
        src_img_corners = np.array(
            [
                [0, 0],
                [self.width, 0],
                [self.width, self.height],
                [0, self.height],
            ],
            dtype=np.float32,
        )

        dst_img_corners = cv2.perspectiveTransform(src_img_corners[None, :, :], mat)[0]
        scale_factor = self.compute_crop_scale(src_img_corners, dst_img_corners)
        return mat, scale_factor

    def compute_crop_scale(self, src_img_corners, dst_img_corners) -> float:
        """
        Compute the crop scale factor.

        Maintain the original image's aspect ratio. Find the closest segment of the warped rectangle to the center of
        the destination image. Using the corresponding corner of the source image, project along this vector to
        this nearest segment. The distance from the center of the source image to this intersection point is the crop
        scale factor.
        """
        dst_center = np.mean(dst_img_corners, axis=0)
        segments = [np.array([dst_img_corners[index], dst_img_corners[(index + 1) % 4]]) for index in range(4)]
        min_index = np.argmin([dist_from_point_to_line(dst_center, segment) for segment in segments])
        closest_segment = segments[min_index]
        src_center = np.mean(src_img_corners, axis=0)
        src_center_to_corner_vector = np.array([src_center, src_img_corners[min_index]])
        src_corner_normalized_vector = src_center_to_corner_vector - src_center
        src_line_in_dst = src_corner_normalized_vector + dst_center
        intersection_point = line_line_intersection(src_line_in_dst, closest_segment)
        dst_corner_magnitude = float(np.linalg.norm(intersection_point - dst_center))
        src_corner_magnitude = float(np.linalg.norm(src_corner_normalized_vector))
        scale_factor = src_corner_magnitude / dst_corner_magnitude
        return scale_factor

    def inverse(self):
        """
        The inverse is to rotate it back with expand, and crop to get the original shape.
        """
        raise NotImplementedError("Inverse not implemented")


class RandomRadialHomography(Augmentation):
    """
    This method returns a copy of this image, skewed by a random homography.
    """

    def __init__(self, min_radius: float, max_radius: float, center: tuple[float, float] = (0.5, 0.5), expand=True):
        assert 0 < min_radius < max_radius
        super().__init__()
        self.center = center
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.expand = expand

    def get_transform(self, image):
        dims = image.shape[:2]
        center = np.array(self.center)
        src_points = (
            np.array(
                [
                    [+self.min_radius, +self.min_radius],
                    [-self.min_radius, +self.min_radius],
                    [-self.min_radius, -self.min_radius],
                    [+self.min_radius, -self.min_radius],
                ]
            )
        ).astype(np.float32)
        dst_points = src_points + np.sign(src_points) * np.random.uniform(
            0.0, self.max_radius - self.min_radius, size=src_points.shape
        )
        dst_points[:, 0] += center[0]
        dst_points[:, 1] += center[1]
        src_points[:, 0] += center[0]
        src_points[:, 1] += center[1]

        src_points *= dims
        dst_points *= dims

        src_points = src_points.astype(np.float32)
        dst_points = dst_points.astype(np.float32)

        return HomographyTransform(src_points, dst_points, width=dims[1], height=dims[0], expand=self.expand)
