from __future__ import annotations

import base64
from dataclasses import dataclass, field
from enum import Enum, IntEnum

import numpy as np
import open3d as o3d
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


class CloudFieldName(Enum):
    X = "x"
    Y = "y"
    Z = "z"
    RGB = "rgb"


class CloudFieldType(IntEnum):
    INT8 = 1
    UINT8 = 2
    INT16 = 3
    UINT16 = 4
    INT32 = 5
    UINT32 = 6
    FLOAT32 = 7
    FLOAT64 = 8


FIELD_TYPE_TO_NUMPY = {
    CloudFieldType.INT8: np.int8,
    CloudFieldType.UINT8: np.uint8,
    CloudFieldType.INT16: np.int16,
    CloudFieldType.UINT16: np.uint16,
    CloudFieldType.INT32: np.int32,
    CloudFieldType.UINT32: np.uint32,
    CloudFieldType.FLOAT32: np.float32,
    CloudFieldType.FLOAT64: np.float64,
}


FIELD_TYPE_TO_SIZE = {
    CloudFieldType.INT8: 1,
    CloudFieldType.UINT8: 1,
    CloudFieldType.INT16: 2,
    CloudFieldType.UINT16: 2,
    CloudFieldType.INT32: 4,
    CloudFieldType.UINT32: 4,
    CloudFieldType.FLOAT32: 4,
    CloudFieldType.FLOAT64: 8,
}


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
    colors_uint32 = (colors_uint8[..., 2] << 16) | (colors_uint8[..., 1] << 8) | (colors_uint8[..., 0])

    return colors_uint32


@dataclass
class PointCloud:
    header: Header = field(default_factory=lambda: Header.auto())
    points: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.float32))
    colors: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.uint32))
    is_bigendian: bool = False
    is_dense: bool = False
    type: str = "sensor_msgs/PointCloud2"

    @classmethod
    def from_depth(cls, depth: Image, camera_info: CameraInfo, depth_scale: float = 1000.0) -> PointCloud:
        depth_data = o3d.geometry.Image(depth)
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_data, to_o3d_intrinsics(camera_info), depth_scale=depth_scale
        )
        points = np.asarray(pcd.points).astype(np.float32)
        cloud_data = points.reshape((camera_info.height, camera_info.width))
        return PointCloud(header=camera_info.header, points=cloud_data)

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
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_data, depth_data, convert_rgb_to_intensity=False, depth_scale=depth_scale
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, to_o3d_intrinsics(camera_info), project_valid_depth_only=False
        )
        points = np.asarray(pcd.points).astype(np.float32).reshape((camera_info.height, camera_info.width, -1))
        colors = to_uint32_color(np.asarray(pcd.colors).astype(np.float32)).reshape(
            (camera_info.height, camera_info.width)
        )
        return PointCloud(header=camera_info.header, points=points, colors=colors)

    def to_raw(self) -> RawRosMessage:
        height, width = self.points.shape[:2]
        msg_fields = []
        if len(self.colors) == 0:
            fields = {
                CloudFieldName.X: CloudFieldType.FLOAT32,
                CloudFieldName.Y: CloudFieldType.FLOAT32,
                CloudFieldName.Z: CloudFieldType.FLOAT32,
            }
        else:
            fields = {
                CloudFieldName.X: CloudFieldType.FLOAT32,
                CloudFieldName.Y: CloudFieldType.FLOAT32,
                CloudFieldName.Z: CloudFieldType.FLOAT32,
                CloudFieldName.RGB: CloudFieldType.UINT32,
            }
        point_step = 0
        for index, (sub_field, field_type) in enumerate(fields.items()):
            type_size = FIELD_TYPE_TO_SIZE[field_type]
            msg_fields.append(
                {
                    "name": sub_field.value,
                    "offset": type_size * index,
                    "datatype": field_type,
                    "count": 1,
                }
            )
            point_step += type_size
        row_step = point_step * width

        cloud_data = self.points
        if len(self.colors) > 0:
            cloud_dtype = np.dtype(
                {
                    "names": ["x", "y", "z", "rgb"],
                    "formats": [np.float32, np.float32, np.float32, np.uint32],
                    "offsets": [0, 4, 8, 12],
                }
            )
            cloud_data = np.empty((height, width), dtype=cloud_dtype)
            cloud_data["x"] = self.points[..., 0]
            cloud_data["y"] = self.points[..., 1]
            cloud_data["z"] = self.points[..., 2]
            cloud_data["rgb"] = self.colors
            print(cloud_data)

        cloud_bytes = base64.b64encode(cloud_data.tobytes()).decode("ascii")

        return {
            "header": self.header.to_raw(),
            "height": height,
            "width": width,
            "fields": msg_fields,
            "is_bigendian": bool(self.is_bigendian),
            "point_step": point_step,
            "row_step": row_step,
            "data": cloud_bytes,
            "is_dense": bool(self.is_dense),
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> PointCloud:
        header = Header.from_raw(msg["header"])
        height = int(msg["height"])
        width = int(msg["width"])
        data: str = msg["data"]
        fields = []
        for sub_field in msg["fields"]:
            fields.append(CloudFieldName(sub_field["name"]))
        if len(fields) == 3:
            assert fields == [CloudFieldName.X, CloudFieldName.Y, CloudFieldName.Z]
        else:
            assert fields == [CloudFieldName.X, CloudFieldName.Y, CloudFieldName.Z, CloudFieldName.RGB]
        is_bigendian = bool(msg["is_bigendian"])

        buffer_dtype = FIELD_TYPE_TO_NUMPY[msg["fields"][0]["datatype"]]
        base64_bytes = data.encode("ascii")
        buffer = np.frombuffer(base64.b64decode(base64_bytes), dtype=buffer_dtype)

        cloud = buffer.reshape((height, width, len(fields)))
        points = cloud[..., :3].astype(np.float32)
        colors = cloud[..., 3:].astype(np.uint32)
        is_dense = bool(msg["is_dense"])

        return PointCloud(header, points, colors, is_bigendian, is_dense)
