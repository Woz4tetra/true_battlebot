from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from perception_tools.messages.geometry.quaternion import Quaternion
from perception_tools.messages.geometry.rpy import RPY
from perception_tools.messages.geometry.xyz import XYZ
from perception_tools.rosbridge.types import RawRosMessage
from scipy.spatial.transform import Rotation


@dataclass
class Transform:
    """
    A class for defining frames.
    """

    tfmat: np.ndarray = field(repr=False)

    @classmethod
    def identity(cls) -> Transform:
        return cls(np.identity(4))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Transform):
            raise NotImplementedError

        return np.allclose(np.array(self), np.array(other))

    @property
    def position(self) -> XYZ:
        """
        The position of the center of the Transform
        """
        return XYZ(self.x, self.y, self.z)

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
    def rotation(self) -> Rotation:
        return Rotation.from_matrix(self.rotation_matrix)

    @property
    def quaternion(self) -> Quaternion:
        """
        Quaternion representing orientation of this transform in space frame
        """
        return Quaternion(*self.rotation.as_quat(canonical=False))

    @property
    def rpy(self) -> RPY:
        """
        RPY representing rpy corresponding to this transform in space frame
        """
        return RPY(*self.rotation.as_euler("xyz", degrees=False))

    @property
    def x(self) -> float:
        return self.tfmat[0, 3]

    @property
    def y(self) -> float:
        return self.tfmat[1, 3]

    @property
    def z(self) -> float:
        return self.tfmat[2, 3]

    def transform_by(self, transform: Transform) -> Transform:
        """
        Apply a second transform in series
        """
        return Transform(np.dot(self.tfmat, transform.tfmat))

    def forward_by(self, transform: Transform) -> Transform:
        """
        Forward this transform by the other transform in series
        """
        return Transform(np.dot(transform.tfmat, self.tfmat))

    def relative_to(self, other: Transform) -> Transform:
        return Transform(np.dot(other.inverse().tfmat, self.tfmat))

    def inverse(self) -> Transform:
        """
        Calculate the inverse of the Transform, creating a new transform that goes from the
        Transform's reference frame to the base reference frame.
        """
        inv_tfmat = np.eye(4)
        inv_tfmat[0:3, 0:3] = self.tfmat[0:3, 0:3].T
        inv_tfmat[0:3, 3] = -inv_tfmat[0:3, 0:3] @ self.tfmat[0:3, 3]
        return Transform(inv_tfmat)

    @classmethod
    def from_position_and_quaternion(
        cls,
        position: XYZ = XYZ(0.0, 0.0, 0.0),
        orientation: Quaternion = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
    ) -> Transform:
        """
        Combine orientation matrix and position vector into Transform

        args
            orientation: 3x3 rotation matrix
            position: 3 element position vector
        """
        rotate_mat = Rotation.from_quat((orientation.x, orientation.y, orientation.z, orientation.w))
        tfmat = np.eye(4)
        tfmat[0:3, 0:3] = rotate_mat.as_matrix()
        tfmat[0:3, 3] = np.array([position.x, position.y, position.z])
        return cls(tfmat)

    @classmethod
    def from_position_and_rpy(cls, position: XYZ = XYZ(0.0, 0.0, 0.0), rpy: RPY = RPY(0.0, 0.0, 0.0)) -> Transform:
        """
        Combine rpy and position vector into Transform
        args:
            position: 3 element position vector
            rpy: 3 element roll pitch yaw, specified in space frame ('sxyz') order
        """
        rotate_mat = Rotation.from_euler("xyz", (rpy.roll, rpy.pitch, rpy.yaw), degrees=False)
        tfmat = np.eye(4)
        tfmat[0:3, 0:3] = rotate_mat.as_matrix()
        tfmat[0:3, 3] = np.array([position.x, position.y, position.z])
        return Transform(tfmat)

    def to_raw(self) -> RawRosMessage:
        return {
            "position": self.position.to_raw(),
            "orientation": self.quaternion.to_raw(),
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Transform:
        return cls.from_position_and_quaternion(XYZ.from_raw(msg["position"]), Quaternion.from_raw(msg["orientation"]))

    def __hash__(self) -> int:
        return hash(tuple([tuple(row) for row in self.tfmat]))

    def __str__(self) -> str:
        return f"{self.__class__.__name__}.from_position_and_rpy(Vector3({self.x, self.y, self.z}), RPY({self.rpy}))"

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.tfmat.tolist()})"


Pose = Transform
