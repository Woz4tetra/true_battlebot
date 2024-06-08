import json
import os
import shutil
from typing import List, Tuple

import cv2
import detectron2.data.datasets
import numpy as np
from bw_shared.geometry.in_plane import dist_from_point_to_line, line_line_intersection
from coco_dataset import CocoMetaDataset, DatasetAnnotation, DatasetImage
from detectron2.data import DatasetCatalog, transforms
from detectron2.data.transforms.augmentation import Augmentation, Transform
from matplotlib import pyplot as plt


def load_dataset(dataset_path: str) -> CocoMetaDataset:
    with open(dataset_path, "r") as f:
        dataset = json.load(f)
    return CocoMetaDataset.from_json(dataset)


def load_coco(dataset_name: str, annotation_path: str, image_dir: str):
    detectron2.data.datasets.register_coco_instances(
        name=dataset_name,
        metadata={},
        json_file=annotation_path,
        image_root=image_dir,
    )
    return DatasetCatalog.get(dataset_name)


def plot_annotated_image(image: np.ndarray, annotations: List[DatasetAnnotation]):
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    for annotation in annotations:
        box = annotation.bbox
        coords = [
            box[0],
            box[1],
            box[0] + box[2],
            box[1] + box[3],
        ]
        for segmentation in annotation.segmentation:
            segmentation = np.array(segmentation).reshape(-1, 2)
            segmentation = np.append(segmentation, [segmentation[0]], axis=0)
            line = plt.plot(segmentation[:, 0], segmentation[:, 1])
            plt.plot([coords[0], coords[2]], [coords[1], coords[3]], color=line[0].get_color())


def copy_dataset(dataset_path: str, destination_path: str) -> None:
    shutil.rmtree(destination_path, ignore_errors=True)
    shutil.copytree(dataset_path, destination_path)


def write_augmented_image(
    image: np.ndarray, new_directory: str, original_image_path: str, augmentation_num: int
) -> str:
    old_image_path, extension = os.path.splitext(original_image_path)
    image_name = os.path.basename(old_image_path)
    new_image_filename = image_name + f"_augment-{augmentation_num:03d}{extension}"
    image_path = os.path.join(new_directory, new_image_filename)
    cv2.imwrite(image_path, image)
    return new_image_filename


def write_dataset(dataset: CocoMetaDataset, path: str) -> None:
    with open(path, "w") as f:
        json.dump(dataset.to_json(), f)


def augment_dataset_image(
    image_path: str, dataset: CocoMetaDataset, dataset_image: DatasetImage, augmentations: transforms.AugmentationList
) -> Tuple[np.ndarray, List[DatasetAnnotation]]:
    annotations = dataset.get_annotations(dataset_image.id)

    image = cv2.imread(image_path)

    aug_input = transforms.AugInput(image)
    aug_transform = augmentations(aug_input)
    image_transformed = aug_input.image
    assert image_transformed is not None

    transformed_annotations = []

    for annotation in annotations:
        segmentations = []
        for segmentation in annotation.segmentation:
            segmentations.append(np.array(segmentation).reshape(-1, 2))
        polygons_transformed = np.array(aug_transform.apply_polygons(segmentations))

        all_coords = []
        for segmentation in polygons_transformed:
            coords = [
                np.min(segmentation[:, 0]),
                np.min(segmentation[:, 1]),
                np.max(segmentation[:, 0]),
                np.max(segmentation[:, 1]),
            ]
            all_coords.append(coords)
        fitted_coords = [
            np.min([bbox[0] for bbox in all_coords]),
            np.min([bbox[1] for bbox in all_coords]),
            np.max([bbox[2] for bbox in all_coords]),
            np.max([bbox[3] for bbox in all_coords]),
        ]
        bbox = [
            fitted_coords[0],
            fitted_coords[1],
            fitted_coords[2] - fitted_coords[0],
            fitted_coords[3] - fitted_coords[1],
        ]

        transformed_annotation = DatasetAnnotation(
            id=-1,
            image_id=-1,
            category_id=annotation.category_id,
            segmentation=[polygons_transformed.flatten().tolist()],
            area=annotation.area,
            bbox=bbox,
            iscrowd=annotation.iscrowd,
        )
        transformed_annotations.append(transformed_annotation)

    return image_transformed, transformed_annotations


class HomographyTransform(Transform):
    def __init__(self, src_corners, dst_corners, width: float, height: float, interp=None, expand=True):
        super().__init__()

        self.expand = expand
        self.interp = interp
        self.width = width
        self.height = height
        self.tf_coords, self.scale_factor = self.perspective_matrix(src_corners, dst_corners)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        self = cls.__new__(cls)
        self.tf_coords = matrix
        return self

    def apply_image(self, img, interp=None):
        """
        img should be a numpy array, formatted as Height * Width * Nchannels
        """
        width = img.shape[1]
        height = img.shape[0]
        warped = cv2.warpPerspective(img, self.tf_coords, (width, height), flags=interp)
        if not self.expand:
            return warped
        width_delta = abs(int((self.scale_factor * width - width) / 2))
        height_delta = abs(int((self.scale_factor * height - height) / 2))
        if self.scale_factor > 1:
            expanded = cv2.copyMakeBorder(
                warped, height_delta, height_delta, width_delta, width_delta, cv2.BORDER_CONSTANT
            )
        else:
            expanded = warped[height_delta:-height_delta, width_delta:-width_delta]
        scaled = cv2.resize(expanded, (width, height))
        return scaled

    def apply_coords(self, coords):
        """
        coords should be a N * 2 array-like, containing N couples of (x, y) points
        """
        coords = np.asarray(coords, dtype=float)
        return cv2.transform(coords[:, np.newaxis, :], self.tf_coords)[:, 0, :]

    def apply_segmentation(self, segmentation):
        segmentation = self.apply_image(segmentation, interp=self.interp)
        return segmentation

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
        src_img_edges = np.array(
            [
                [self.width, self.height / 2],
                [self.width / 2, self.height],
                [0, self.height / 2],
                [self.width / 2, 0],
            ],
            dtype=np.float32,
        )

        dst_img_corners = cv2.perspectiveTransform(src_img_corners[None, :, :], mat)[0]
        scale_factor = self.compute_crop_scale(src_img_edges, dst_img_corners)
        return mat, scale_factor

    def compute_crop_scale(self, src_img_edges, dst_img_corners) -> float:
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
        src_center = np.mean(src_img_edges, axis=0)
        src_edge_normalized_vector = src_img_edges[min_index] - src_center
        src_line_in_dst = src_edge_normalized_vector + dst_center
        intersection_point = line_line_intersection(src_line_in_dst, closest_segment)
        dst_magnitude = float(np.linalg.norm(intersection_point - dst_center))
        src_magnitude = float(np.linalg.norm(src_edge_normalized_vector))
        scale_factor = src_magnitude / dst_magnitude
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

    def __init__(
        self, min_radius: float, max_radius: float, center: tuple[float, float] = (0.5, 0.5), interp=None, expand=True
    ):
        super().__init__()
        self.center = center
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.interp = interp
        self.expand = expand

    def get_transform(self, image):
        dims = image.shape[:2]
        center = np.array(self.center)
        src_points = (
            np.array(
                [
                    [center[0] + self.min_radius, center[1] + self.min_radius],
                    [center[0] - self.min_radius, center[1] + self.min_radius],
                    [center[0] - self.min_radius, center[1] - self.min_radius],
                    [center[0] + self.min_radius, center[1] - self.min_radius],
                ]
            )
        ).astype(np.float32)
        normalized_src = src_points - center
        normalized_src += np.random.uniform(0.0, self.max_radius - self.min_radius, size=normalized_src.shape)
        dst_points = normalized_src + center

        src_points *= dims
        dst_points *= dims

        src_points = src_points.astype(np.float32)
        dst_points = dst_points.astype(np.float32)

        return HomographyTransform(
            src_points, dst_points, width=dims[1], height=dims[0], interp=self.interp, expand=self.expand
        )
