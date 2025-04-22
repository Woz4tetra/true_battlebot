from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from geometry_msgs.msg import Transform as RosTransform

from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.rotation_transforms import (
    euler_from_matrix,
    euler_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
)
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.xyz import XYZ


@dataclass(frozen=True)
class Transform3D:
    """
    A class for defining frames.
    """

    tfmat: np.ndarray = field(repr=False)

    @classmethod
    def identity(cls) -> Transform3D:
        return cls(np.identity(4))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Transform3D):
            return False

        return bool(np.all(self.tfmat == other.tfmat))

    @property
    def position(self) -> Vector3:
        """
        The position of the center of the Transform
        """
        return Vector3(self.x, self.y, self.z)

    @property
    def point(self) -> Point:
        """
        The position of the center of the Transform
        """
        return Point(self.x, self.y, self.z)

    @property
    def position_array(self) -> np.ndarray:
        """
        The position of the center of the Transform as a numpy array
        """
        return self.tfmat[0:3, 3]

    @property
    def rotation_matrix(self) -> np.ndarray:
        """
        3x3 array where the column vectors give the x, y, and z axis of the Transform
        """
        return self.tfmat[0:3, 0:3]

    @property
    def quaternion(self) -> Quaternion:
        """
        Quaternion representing orientation of this transform in space frame
        """
        return quaternion_from_matrix(self.tfmat)

    @property
    def quaternion_np(self) -> np.ndarray:
        """
        Quaternion array representing orientation of this transform in space frame
        """
        return np.array([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])

    @property
    def rpy(self) -> RPY:
        """
        RPY representing rpy corresponding to this transform in space frame
        """
        angles = euler_from_matrix(self.tfmat)
        return RPY(angles)

    @property
    def x(self) -> float:
        return self.tfmat[0, 3]

    @property
    def y(self) -> float:
        return self.tfmat[1, 3]

    @property
    def z(self) -> float:
        return self.tfmat[2, 3]

    def forward_by(self, transform: Transform3D) -> Transform3D:
        """
        Apply a second transform in series
        """
        return Transform3D(np.dot(self.tfmat, transform.tfmat))

    def transform_by(self, transform: Transform3D) -> Transform3D:
        """
        Transform by the other transform in series
        """
        return Transform3D(np.dot(transform.tfmat, self.tfmat))

    def relative_to(self, other: Transform3D) -> Transform3D:
        return Transform3D(np.dot(other.inverse().tfmat, self.tfmat))

    def inverse(self) -> Transform3D:
        """
        Calculate the inverse of the Transform, creating a new transform that goes from the
        Transform's reference frame to the base reference frame.
        """
        inv_tfmat = np.eye(4)
        inv_tfmat[0:3, 0:3] = self.tfmat[0:3, 0:3].T
        inv_tfmat[0:3, 3] = -inv_tfmat[0:3, 0:3] @ self.tfmat[0:3, 3]
        return Transform3D(inv_tfmat)

    def transform_point(self, point: XYZ) -> XYZ:
        """
        Transform a point from the base frame to the transform's frame
        """
        transformed_point = self.tfmat @ np.append(point, 1)
        return XYZ(*transformed_point[0:3])

    @classmethod
    def from_position_and_quaternion(
        cls,
        position: Union[Vector3, Point] = Vector3(),
        orientation: Quaternion = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
    ) -> Transform3D:
        """
        Combine orientation matrix and position vector into Transform

        args
            orientation: 3x3 rotation matrix
            position: 3 element position vector
        """
        tfmat = quaternion_matrix(orientation)
        tfmat[0:3, 3] = np.array([position.x, position.y, position.z])
        return cls(tfmat)

    @classmethod
    def from_position_and_rpy(
        cls, position: Union[Vector3, Point] = Vector3(), rpy: RPY = RPY((0.0, 0.0, 0.0))
    ) -> Transform3D:
        """
        Combine rpy and position vector into Transform
        args:
            position: 3 element position vector
            rpy: 3 element roll pitch yaw, specified in space frame ('sxyz') order
        """
        tfmat = rpy.to_matrix()
        tfmat[0:3, 3] = np.array([position.x, position.y, position.z])
        return Transform3D(tfmat)

    def to_msg(self) -> RosTransform:
        return RosTransform(translation=self.position, rotation=self.quaternion)

    def to_pose_msg(self) -> Pose:
        return Pose(position=self.point, orientation=self.quaternion)

    @classmethod
    def from_msg(cls, msg: RosTransform) -> Transform3D:
        tfmat = quaternion_matrix(msg.rotation)
        tfmat[0:3, 3] = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
        return cls(tfmat)

    @classmethod
    def from_pose_msg(cls, msg: Pose) -> Transform3D:
        tfmat = quaternion_matrix(msg.orientation)
        tfmat[0:3, 3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        return cls(tfmat)

    @classmethod
    def from_pose2d(cls, pose2d: Pose2D) -> Transform3D:
        tfmat = euler_matrix(RPY((0.0, 0.0, pose2d.theta)))
        tfmat[0, 3] = pose2d.x
        tfmat[1, 3] = pose2d.y
        return Transform3D(tfmat)

    def to_pose2d(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.rpy[2])

    def to_xyzquat(self) -> tuple[float, float, float, float, float, float, float]:
        position = self.position
        orientation = self.quaternion
        return position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w

    def almost_equal(self, other: Transform3D, atol: float = 1e-6, rtol: float = 1e-6) -> bool:
        return np.allclose(self.tfmat, other.tfmat, atol=atol, rtol=rtol)

    def __hash__(self) -> int:
        return hash(tuple([tuple(row) for row in self.tfmat]))

    def __str__(self) -> str:
        rpy_deg = tuple(np.rad2deg(self.rpy).tolist())
        return (
            f"{self.__class__.__name__}.from_position_and_rpy(Vector3({self.x}, {self.y}, {self.z}), "
            f"RPY.from_degrees({rpy_deg}))"
        )

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.tfmat.tolist()})"
