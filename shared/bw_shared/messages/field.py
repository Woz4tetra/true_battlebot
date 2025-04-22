from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from bw_interfaces.msg import EstimatedObject

from bw_shared.enums.label import Label
from bw_shared.geometry.field_bounds import FieldBounds, FieldBounds2D
from bw_shared.geometry.plane import Plane
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from bw_shared.geometry.xy import XY
from bw_shared.geometry.xyz import XYZ
from bw_shared.messages.header import Header


@dataclass(frozen=True)
class Field:
    header: Header = field(default_factory=Header.auto)
    child_frame_id: str = ""
    tf_map_from_camera: Transform3D = field(default_factory=Transform3D.identity)
    size: XYZ = field(default_factory=lambda: XYZ(0.0, 0.0, 0.0))
    label: Label = Label.FIELD

    @classmethod
    def from_msg(cls, msg: EstimatedObject) -> Field:
        return cls(
            header=Header.from_msg(msg.header),
            child_frame_id=msg.child_frame_id,
            tf_map_from_camera=Transform3D.from_pose_msg(msg.pose.pose),
            size=XYZ.from_msg(msg.size),
            label=Label(msg.label),
        )

    def to_msg(self) -> EstimatedObject:
        msg = EstimatedObject()
        msg.header = self.header.to_msg()
        msg.pose.pose = self.tf_map_from_camera.to_pose_msg()
        msg.child_frame_id = self.child_frame_id
        msg.size = self.size.to_msg()
        msg.label = self.label.value
        return msg

    @cached_property
    def tf_camera_from_map(self) -> Transform3D:
        return self.tfstamped_camera_from_map.transform

    @cached_property
    def tfstamped_camera_from_map(self) -> Transform3DStamped:
        return self.to_transform_stamped().inverse()

    @cached_property
    def plane_in_map(self) -> Plane:
        return Plane.from_transform(self.tf_map_from_camera)

    @cached_property
    def plane_in_camera(self) -> Plane:
        return Plane.from_transform(self.tf_camera_from_map)

    @cached_property
    def bounds(self) -> FieldBounds:
        half_x = self.size.x / 2
        half_y = self.size.y / 2
        return (
            XYZ(-half_x, -half_y, 0.0),
            XYZ(half_x, half_y, self.size.z),
        )

    @cached_property
    def bounds_2d(self) -> FieldBounds2D:
        return (
            XY(self.bounds[0].x, self.bounds[0].y),
            XY(self.bounds[1].x, self.bounds[1].y),
        )

    @cached_property
    def corners(self) -> np.ndarray:
        """Return the corners of the field in the map frame."""
        return np.array(
            [
                [self.bounds[0].x, self.bounds[0].y, self.bounds[0].z],
                [self.bounds[1].x, self.bounds[0].y, self.bounds[0].z],
                [self.bounds[1].x, self.bounds[1].y, self.bounds[0].z],
                [self.bounds[0].x, self.bounds[1].y, self.bounds[0].z],
            ]
        )

    @cached_property
    def corners_in_camera(self) -> np.ndarray:
        return points_transform_by(self.corners, self.tf_camera_from_map.tfmat)

    def to_transform_stamped(self) -> Transform3DStamped:
        return Transform3DStamped(
            header=self.header, child_frame_id=self.child_frame_id, transform=self.tf_map_from_camera
        )
