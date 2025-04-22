from __future__ import annotations

from functools import cached_property
from typing import Iterable, Optional

import numpy as np
from geometry_msgs.msg import Vector3

from bw_shared.epsilon import EPSILON
from bw_shared.geometry.array_conversions import PointOrVector, xyz_to_array
from bw_shared.geometry.projection_math.find_ray_plane_intersection import find_ray_plane_intersection
from bw_shared.geometry.projection_math.plane_from_3_points import plane_from_3_points
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import transform_matrix_from_vectors
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform_to_plane import transform_to_plane


def xyz_to_str(xyz: PointOrVector) -> str:
    """Convert a Point or Vector3 to a string."""
    return f"{xyz.x}, {xyz.y}, {xyz.z}"


class Plane:
    def __init__(self, center: PointOrVector, normal: PointOrVector) -> None:
        self._center = center
        self._normal = normal

    @property
    def center(self) -> PointOrVector:
        """Return the center of the plane."""
        return self._center

    @property
    def normal(self) -> PointOrVector:
        """Return the normal of the plane."""
        return self._normal

    @classmethod
    def from_iterable(cls, center: Iterable[float], normal: Iterable[float]) -> Plane:
        return cls(Vector3(*center), Vector3(*normal))

    @classmethod
    def from_transform(cls, transform: Transform3D) -> Plane:
        center, normal = transform_to_plane(transform.tfmat)
        return cls(Vector3(*center), Vector3(*normal))

    @classmethod
    def from_3_point_arrays(cls, point1: np.ndarray, point2: np.ndarray, point3: np.ndarray) -> Plane:
        plane_tri_points = np.array([point1, point2, point3])
        plane_normal = plane_from_3_points(point1, point2, point3)
        plane_center = np.mean(plane_tri_points, axis=0)
        return cls(Vector3(*plane_center), Vector3(*plane_normal))

    @classmethod
    def from_3_points(cls, point1: PointOrVector, point2: PointOrVector, point3: PointOrVector) -> Plane:
        return cls.from_3_point_arrays(xyz_to_array(point1), xyz_to_array(point2), xyz_to_array(point3))

    def to_transform(self, up_vec: Optional[np.ndarray] = None, epsilon: float = EPSILON) -> Transform3D:
        return transform_matrix_from_vectors(
            xyz_to_array(self.center), xyz_to_array(self.normal), up_vec=up_vec, epsilon=epsilon
        )

    def invert_normal(self) -> Plane:
        """Return a new Plane with the normal inverted."""
        return Plane(self.center, Vector3(-self.normal.x, -self.normal.y, -self.normal.z))

    @cached_property
    def normal_array(self) -> np.ndarray:
        """Return the normal as a numpy array."""
        return xyz_to_array(self.normal)

    @cached_property
    def center_array(self) -> np.ndarray:
        """Return the center as a numpy array."""
        return xyz_to_array(self.center)

    def ray_intersection(self, ray: Vector3) -> Vector3:
        return Vector3(*find_ray_plane_intersection(xyz_to_array(ray), self.center_array, self.normal_array))

    def __repr__(self) -> str:
        return f"Plane(center=Vector3({xyz_to_str(self.center)}), normal=Vector3({xyz_to_str(self.normal)}))"

    __str__ = __repr__
