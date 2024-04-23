from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData as RosMapMetaData
from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
from numpy import typing as npt

from bw_tools.structs.header import Header
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.transform3d import Transform3D

MapImage = npt.NDArray[np.uint8]  # MxN
MapBytes = npt.NDArray[np.int8]  # (M*N)x1
MapBytesImage = npt.NDArray[np.int8]  # MxN


@dataclass(frozen=True, eq=True)
class MapMetaData:
    map_load_time: float
    resolution: float
    width: int
    height: int
    origin: Pose2D

    @classmethod
    def from_msg(cls, msg: RosMapMetaData) -> MapMetaData:
        return cls(
            map_load_time=msg.map_load_time.to_sec(),
            resolution=msg.resolution,
            width=msg.width,
            height=msg.height,
            origin=Pose2D.from_msg(msg.origin),
        )

    def to_msg(self) -> RosMapMetaData:
        return RosMapMetaData(
            map_load_time=rospy.Time.from_sec(self.map_load_time),
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin=self.origin.to_msg(),
        )

    @classmethod
    def from_other(cls, other: MapMetaData) -> MapMetaData:
        return cls(
            map_load_time=other.map_load_time,
            resolution=other.resolution,
            width=other.width,
            height=other.height,
            origin=Pose2D(other.origin.x, other.origin.y, other.origin.theta),
        )

    def get_world_x_y(self, costmap_x: int, costmap_y: int) -> Tuple[float, float]:
        world_x = costmap_x * self.resolution + self.origin.x
        world_y = (self.height - costmap_y) * self.resolution + self.origin.y
        return world_x, world_y

    def get_costmap_x_y(self, world_x: float, world_y: float) -> Tuple[int, int]:
        costmap_x = int(round((world_x - self.origin.x) / self.resolution))
        costmap_y = int(self.height - round((world_y - self.origin.y) / self.resolution))

        return costmap_x, costmap_y

    def get_grid_data_index(self, x: int, y: int) -> int:
        return y * self.width + x


@dataclass(frozen=True, eq=True)
class OccupancyGrid:
    header: Header
    info: MapMetaData
    data: MapBytes

    @cached_property
    def image(self) -> MapBytesImage:
        return np.reshape(self.data, (self.info.height, self.info.width))

    @classmethod
    def from_msg(cls, msg: RosOccupancyGrid) -> OccupancyGrid:
        return cls(
            header=Header.from_msg(msg.header),
            info=MapMetaData.from_msg(msg.info),
            data=np.array(msg.data, dtype=np.int8),
        )

    def to_msg(self) -> RosOccupancyGrid:
        return RosOccupancyGrid(
            header=self.header.to_msg(),
            info=self.info.to_msg(),
            data=self.data.astype(np.int8).tolist(),
        )

    @classmethod
    def from_other(cls, other: OccupancyGrid) -> OccupancyGrid:
        return cls(
            header=Header(other.header.stamp, other.header.frame_id, other.header.seq),
            info=MapMetaData.from_other(other.info),
            data=np.copy(other.data),
        )

    @classmethod
    def make_empty(cls) -> OccupancyGrid:
        return cls(
            header=Header.auto(),
            info=MapMetaData(
                map_load_time=0.0,
                resolution=1.0,
                width=0,
                height=0,
                origin=Pose2D(0.0, 0.0, 0.0),
            ),
            data=np.array([], dtype=np.int8),
        )

    @classmethod
    def from_image(
        cls,
        header: Header,
        image: MapBytesImage,
        resolution: float,
        center: Optional[Transform3D] = None,
    ) -> OccupancyGrid:
        px_height = image.shape[0]
        px_width = image.shape[1]
        width = px_width * resolution
        height = px_height * resolution
        origin = Pose2D(-width / 2.0, -height / 2.0, 0.0)
        if center is not None:
            origin = Transform3D.from_pose2d(origin).transform_by(center).to_pose2d()
        metadata = MapMetaData(
            map_load_time=0.0,
            resolution=resolution,
            width=px_width,
            height=px_height,
            origin=origin,
        )
        data = image.flatten()
        return cls(header, metadata, data)

    @classmethod
    def from_map_file(
        cls, header: Header, config: Dict[str, Any], image: MapImage, map_load_time: float = 0.0
    ) -> OccupancyGrid:
        """
        This method creates an OccupancyGridManager using map file data.

        map files encode free, occupied, or unknown areas. When opening the image
        in a normal image viewer, the +Y axis is down and +X axis is to the right.
        To view map files, the image needs to be flipped along the X axis. This method
        does not flip the image.

        grid_data schema:
            free = 0
            occupied = 100
            unknown = -1
        map file schema (default gmapping output):
            free = 254 (occupied if negate is True)
            occupied = 0 (free if negate is True)
            unknown = 205
        map -> grid_data transform:
            convert BGR to gray
            bitwise not
            scale from 0..255 to 0..1
            label pixels < free_thresh as 0 (free)
            label pixels > occupied_thresh as 100 (occupied)
            label other pixels as -1 (unknown)
        """

        width = image.shape[1]
        height = image.shape[0]

        resolution: float = config["resolution"]
        origin: Tuple[float, float, float] = config["origin"]

        occupied_thresh: float = config["occupied_thresh"]
        free_thresh: float = config["free_thresh"]
        negate = config["negate"] != 0
        if negate:
            swap = occupied_thresh
            occupied_thresh = free_thresh
            free_thresh = swap

        max_value = np.iinfo(np.uint8).max
        float_image = np.bitwise_not(image).astype(np.float32)
        float_image /= max_value
        grid_image = np.zeros(float_image.shape, dtype=np.int8)
        grid_image[np.where(float_image < free_thresh)] = 0
        grid_image[np.where(float_image > occupied_thresh)] = 100
        grid_image[np.where((float_image > free_thresh) & (float_image < occupied_thresh))] = -1

        return cls(
            header=header,
            info=MapMetaData(
                map_load_time=map_load_time,
                resolution=resolution,
                width=width,
                height=height,
                origin=Pose2D(origin[0], origin[1], origin[2]),
            ),
            data=grid_image.flatten(),
        )

    def to_map_file(
        self,
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
        unknown: float = 0.2,
        negate: bool = False,
    ) -> Tuple[Dict[str, Any], MapImage]:
        config = {
            "resolution": self.info.resolution,
            "origin": self.info.origin.to_tuple(),
            "occupied_thresh": occupied_thresh,
            "free_thresh": free_thresh,
            "negate": int(negate),
        }
        max_value = np.iinfo(np.uint8).max
        unknown_value = int(max_value * (1.0 - unknown))

        max_value = np.iinfo(np.uint8).max
        image = self.image.astype(np.float32)
        image = image * max_value / 100.0
        map_image = image.astype(np.uint8)
        map_image = np.bitwise_not(image)
        map_image[self.image <= -1] = unknown_value
        map_image[self.image > 100] = unknown_value

        return config, map_image

    def to_costmap_file(self) -> Tuple[Dict[str, Any], MapImage]:
        config = {
            "resolution": self.info.resolution,
            "origin": self.info.origin.to_tuple(),
            "occupied_thresh": 0.0,  # unused by costmap
            "free_thresh": 0.0,  # unused by costmap
            "negate": 0,  # unused by costmap
        }
        unknown_value = 0

        image = self.image.astype(np.uint8)
        image = np.bitwise_not(image)
        image[self.image <= -1] = unknown_value
        image[self.image > 100] = unknown_value

        return config, image

    def to_debug_image(self, unknown_color: Tuple[float, float, float] = (128, 128, 0)) -> MapImage:
        max_value = np.iinfo(np.uint8).max
        image = self.image.astype(np.float32)
        image = image / 100.0 * max_value
        map_image = image.astype(np.uint8)
        map_image = np.bitwise_not(map_image)
        if map_image.size > 0:
            map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR).astype(np.uint8)
            map_image[np.where(self.image < 0)] = np.array(unknown_color, dtype=np.uint8)
        else:
            return np.array([], dtype=np.uint8)
        map_image = np.flipud(map_image)
        map_image = np.ascontiguousarray(map_image, dtype=np.uint8)
        return map_image

    def get_cost_from_world_x_y(self, x: float, y: float) -> int:
        return self.get_cost_from_costmap_x_y(*self.info.get_costmap_x_y(x, y))

    def get_cost_from_costmap_x_y(self, x: int, y: int) -> int:
        return int(self.data[self.info.get_grid_data_index(x, y)])

    def set_scale(self, scale: float) -> OccupancyGrid:
        if abs(scale - 1.0) < 1e-8:
            return OccupancyGrid.from_other(self)

        resolution = self.info.resolution / scale
        width = int(self.info.width * scale)
        height = int(self.info.height * scale)
        metadata = MapMetaData(
            map_load_time=self.info.map_load_time,
            resolution=resolution,
            width=width,
            height=height,
            origin=Pose2D(self.info.origin.x, self.info.origin.y, self.info.origin.theta),
        )
        image: MapBytesImage = np.array(cv2.resize(self.image, dsize=(width, height), interpolation=cv2.INTER_NEAREST))
        data = image.flatten()
        return OccupancyGrid(self.header, metadata, data)
