from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from geometry_msgs.msg import Transform as RosTransform
from tf_conversions import transformations

from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.rpy import RPY


@dataclass
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
            raise NotImplementedError

        return np.allclose(np.array(self), np.array(other))

    @property
    def position(self) -> Vector3:
        """
        The position of the center of the Transform
        """
        return Vector3(self.x, self.y, self.z)

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
        return Quaternion(*transformations.quaternion_from_matrix(self.tfmat))

    @property
    def rpy(self) -> RPY:
        """
        RPY representing rpy corresponding to this transform in space frame
        """
        angles = transformations.euler_from_matrix(self.tfmat)
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

    def transform_by(self, transform: Transform3D) -> Transform3D:
        """
        Apply a second transform in series
        """
        return Transform3D(np.dot(self.tfmat, transform.tfmat))

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
        return Transform3D(inv_tfmat)  # type: ignore

    @classmethod
    def from_position_and_quaternion(cls, position: Vector3, orientation: Quaternion) -> Transform3D:
        """
        Combine orientation matrix and position vector into Transform

        args
            orientation: 3x3 rotation matrix
            position: 3 element position vector
        """
        tfmat = transformations.quaternion_matrix((orientation.x, orientation.y, orientation.z, orientation.w))
        tfmat[0:3, 3] = np.array([position.x, position.y, position.z])
        return cls(tfmat)

    @classmethod
    def from_position_and_rpy(cls, position: Vector3, rpy: RPY) -> Transform3D:
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
        return RosTransform(
            translation=self.position,
            rotation=self.quaternion,
        )

    @classmethod
    def from_msg(cls, msg: RosTransform) -> Transform3D:
        orientation_mat = transformations.quaternion_matrix(
            (msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
        )
        tfmat = np.eye(4)
        tfmat[0:3, 0:3] = orientation_mat
        tfmat[0:3, 3] = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
        return cls(tfmat)

    @classmethod
    def from_pose2d(cls, pose2d: Pose2D) -> Transform3D:
        tfmat = np.eye(4)
        tfmat[0:3, 0:3] = transformations.euler_matrix(0.0, 0.0, pose2d.theta)
        tfmat[0, 0] = pose2d.x
        tfmat[0, 1] = pose2d.y
        return Transform3D(tfmat)

    def to_pose2d(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.rpy[2])

    def __hash__(self) -> int:
        return hash(tuple([tuple(row) for row in self.tfmat]))

    def __str__(self) -> str:
        return (
            f"{self.__class__.__name__}.from_position_and_rpy(" f"Vector3({self.x, self.y, self.z}), RPY({self.rpy}))"
        )

    def ___repr__(self) -> str:
        return f"{self.__class__.__name__}({self.tfmat})"
