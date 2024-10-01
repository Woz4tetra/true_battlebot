from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np
import open3d as o3d
from bw_shared.messages.header import Header
from sensor_msgs.msg import CameraInfo, PointField
from sensor_msgs.msg import PointCloud2 as RosPointCloud

from perception_tools.messages.image import Image


class CloudFieldName(Enum):
    X = "x"
    Y = "y"
    Z = "z"
    RGB = "rgb"
    RGBA = "rgba"
    BGR = "bgr"
    BGRA = "bgra"


FIELD_TYPE_TO_NUMPY = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


FIELD_TYPE_TO_SIZE = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


DUMMY_FIELD_PREFIX = "__"
SUPPORTED_COLOR_FIELDS = ["rgba", "rgb", "bgra", "bgr"]


def to_o3d_intrinsics(camera_info: CameraInfo) -> o3d.camera.PinholeCameraIntrinsic:
    fx = camera_info.K[0]
    cx = camera_info.K[2]
    fy = camera_info.K[4]
    cy = camera_info.K[5]
    return o3d.camera.PinholeCameraIntrinsic(
        width=camera_info.width, height=camera_info.height, fx=fx, fy=fy, cx=cx, cy=cy
    )


def to_uint32_color(colors: np.ndarray) -> np.ndarray:
    # Ensure colors are in the range [0, 1]
    colors = colors / 255.0 if np.max(colors) > 1.0 else colors

    # Convert to uint8 and scale to [0, 255]
    colors_uint8 = (colors * 255).astype(np.uint32)

    # Combine channels into a single uint32 value
    colors_uint32 = (colors_uint8[..., 0] << 16) | (colors_uint8[..., 1] << 8) | (colors_uint8[..., 2])

    return colors_uint32


def from_uint32_color(colors_uint32: np.ndarray, color_order: str) -> np.ndarray:
    # Extract the individual color channels
    if color_order.startswith("rgb"):
        r = colors_uint32 & 0xFF
        g = (colors_uint32 >> 8) & 0xFF
        b = (colors_uint32 >> 16) & 0xFF
    elif color_order.startswith("bgr"):
        r = (colors_uint32 >> 16) & 0xFF
        g = (colors_uint32 >> 8) & 0xFF
        b = colors_uint32 & 0xFF

    # Stack the channels back into an array
    colors_uint8 = np.stack((r, g, b), axis=-1)

    # Convert to float and scale to [0, 1]
    colors = colors_uint8.astype(np.float32) / 255.0

    return colors


def fields_to_dtype(fields: list[PointField], point_step: int) -> dict[str, list[Any]]:
    """Convert a list of PointFields to a numpy record datatype."""
    offset = 0
    type_def: dict[str, list[Any]] = {
        "names": [],
        "formats": [],
        "offsets": [],
    }
    for point_field in fields:
        while offset < point_field.offset:
            # might be extra padding between fields
            type_def["names"].append(f"{DUMMY_FIELD_PREFIX}{offset}")
            type_def["formats"].append(np.uint8)
            type_def["offsets"].append(offset)
            offset += 1

        dtype = FIELD_TYPE_TO_NUMPY[point_field.datatype]
        if point_field.count != 1:
            dtype = np.dtype((dtype, point_field.count))  # type: ignore

        type_def["names"].append(point_field.name)
        type_def["formats"].append(dtype)
        type_def["offsets"].append(point_field.offset)
        offset += FIELD_TYPE_TO_SIZE[point_field.datatype] * point_field.count

    # might be extra padding between points
    while offset < point_step:
        type_def["names"].append(f"{DUMMY_FIELD_PREFIX}{offset}")
        type_def["formats"].append(np.uint8)
        type_def["offsets"].append(offset)
        offset += 1

    return type_def


def rospointcloud_to_array(cloud_msg: RosPointCloud):
    # construct a numpy record type equivalent to the point type of this cloud
    type_def = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)  # type: ignore

    np_dtype = np.dtype(type_def)  # type: ignore
    np_dtype = np_dtype.newbyteorder("<" if cloud_msg.is_bigendian else ">")

    # parse the cloud into an array
    return np.frombuffer(cloud_msg.data, np_dtype)


@dataclass
class PointCloud:
    header: Header = field(default_factory=lambda: Header.auto())
    points: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.float32))
    colors: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.uint32))
    is_bigendian: bool = False
    is_dense: bool = False
    color_encoding: CloudFieldName = CloudFieldName.RGB

    def masked_points(self, mask: np.ndarray) -> np.ndarray:
        filtered_points = np.copy(self.points)
        filtered_points[np.bitwise_not(mask)] = np.nan
        return np.ma.masked_invalid(filtered_points)

    def filtered_points(self, mask: np.ndarray) -> np.ndarray:
        filtered_points = self.points[mask]
        valid_mask = np.all(np.isfinite(filtered_points), axis=-1)
        return filtered_points[valid_mask]

    @classmethod
    def from_depth(cls, depth: Image, camera_info: CameraInfo, depth_scale: float = 1000.0) -> PointCloud:
        depth_data = o3d.geometry.Image(depth.data)
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_data, to_o3d_intrinsics(camera_info), depth_scale=depth_scale
        )
        points = np.asarray(pcd.points).astype(np.float32)
        cloud_data = points.reshape((camera_info.height, camera_info.width))
        return PointCloud(header=Header.from_msg(camera_info.header), points=cloud_data)

    @classmethod
    def from_rgbd(
        cls,
        color: Image,
        depth: Image,
        camera_info: CameraInfo,
        depth_scale: float = 1000.0,
    ) -> PointCloud:
        color_data = o3d.geometry.Image(color.data)
        depth_data = o3d.geometry.Image(depth.data)
        print(np.min(depth.data), np.max(depth.data))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_data, depth_data, convert_rgb_to_intensity=False, depth_scale=depth_scale, depth_trunc=1000.0
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, to_o3d_intrinsics(camera_info), project_valid_depth_only=False
        )
        points = np.asarray(pcd.points).astype(np.float32).reshape((camera_info.height, camera_info.width, -1))
        colors = to_uint32_color(np.asarray(pcd.colors).astype(np.float32)).reshape(
            (camera_info.height, camera_info.width)
        )
        print(np.nanmin(points), np.nanmax(points))
        return PointCloud(
            header=Header.from_msg(camera_info.header), points=points, colors=colors, color_encoding=CloudFieldName.RGB
        )

    def to_msg(self) -> RosPointCloud:
        if len(self.points) == 0:
            return RosPointCloud(header=self.header.to_msg())
        height, width = self.points.shape[:2]
        msg_fields = []
        if len(self.colors) == 0:
            fields = {
                CloudFieldName.X: PointField.FLOAT32,
                CloudFieldName.Y: PointField.FLOAT32,
                CloudFieldName.Z: PointField.FLOAT32,
            }
        else:
            fields = {
                CloudFieldName.X: PointField.FLOAT32,
                CloudFieldName.Y: PointField.FLOAT32,
                CloudFieldName.Z: PointField.FLOAT32,
                self.color_encoding: PointField.UINT32,
            }
        point_step = 0
        for index, (sub_field, field_type) in enumerate(fields.items()):
            type_size = FIELD_TYPE_TO_SIZE[field_type]
            msg_fields.append(
                PointField(
                    name=sub_field.value,
                    offset=type_size * index,
                    datatype=field_type,
                    count=1,
                )
            )
            point_step += type_size
        row_step = point_step * width

        cloud_data = self.points
        if len(self.colors) > 0:
            cloud_dtype = np.dtype(
                {
                    "names": ["x", "y", "z", "color"],
                    "formats": [np.float32, np.float32, np.float32, np.uint32],
                    "offsets": [0, 4, 8, 12],
                }
            )
            cloud_data = np.empty((height, width), dtype=cloud_dtype)
            cloud_data["x"] = self.points[..., 0]
            cloud_data["y"] = self.points[..., 1]
            cloud_data["z"] = self.points[..., 2]
            cloud_data["color"] = self.colors

        cloud_bytes = cloud_data.tobytes()

        return RosPointCloud(
            header=self.header.to_msg(),
            height=height,
            width=width,
            fields=msg_fields,
            is_bigendian=bool(self.is_bigendian),
            point_step=point_step,
            row_step=row_step,
            data=cloud_bytes,
            is_dense=bool(self.is_dense),
        )

    @classmethod
    def from_msg(cls, msg: RosPointCloud) -> PointCloud:
        cloud_array = rospointcloud_to_array(msg)
        x = cloud_array["x"].view(np.float32)
        y = cloud_array["y"].view(np.float32)
        z = cloud_array["z"].view(np.float32)
        points = np.stack((x, y, z), axis=-1)
        colors = np.array([], dtype=np.uint32)
        for color_field in SUPPORTED_COLOR_FIELDS:
            if color_field in cloud_array.dtype.names:  # type: ignore
                colors = from_uint32_color(cloud_array[color_field].view(np.uint32), color_field)
                break
        return PointCloud(
            header=Header.from_msg(msg.header),
            points=points,
            colors=colors,
            is_bigendian=msg.is_bigendian,
            is_dense=msg.is_dense,
        )
