from __future__ import annotations

import base64
import sys
from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from perception_tools.messages.camera.compressed_depth_image import CompressedDepthImage
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


class Encoding(Enum):
    RGB8 = "rgb8"
    RGBA8 = "rgba8"
    RGB16 = "rgb16"
    RGBA16 = "rgba16"
    BGR8 = "bgr8"
    BGRA8 = "bgra8"
    BGR16 = "bgr16"
    BGRA16 = "bgra16"
    MONO8 = "mono8"
    MONO16 = "mono16"

    # OpenCV CvMat types
    TYPE_8UC1 = "8UC1"
    TYPE_8UC2 = "8UC2"
    TYPE_8UC3 = "8UC3"
    TYPE_8UC4 = "8UC4"
    TYPE_8SC1 = "8SC1"
    TYPE_8SC2 = "8SC2"
    TYPE_8SC3 = "8SC3"
    TYPE_8SC4 = "8SC4"
    TYPE_16UC1 = "16UC1"
    TYPE_16UC2 = "16UC2"
    TYPE_16UC3 = "16UC3"
    TYPE_16UC4 = "16UC4"
    TYPE_16SC1 = "16SC1"
    TYPE_16SC2 = "16SC2"
    TYPE_16SC3 = "16SC3"
    TYPE_16SC4 = "16SC4"
    TYPE_32SC1 = "32SC1"
    TYPE_32SC2 = "32SC2"
    TYPE_32SC3 = "32SC3"
    TYPE_32SC4 = "32SC4"
    TYPE_32FC1 = "32FC1"
    TYPE_32FC2 = "32FC2"
    TYPE_32FC3 = "32FC3"
    TYPE_32FC4 = "32FC4"
    TYPE_64FC1 = "64FC1"
    TYPE_64FC2 = "64FC2"
    TYPE_64FC3 = "64FC3"
    TYPE_64FC4 = "64FC4"

    # Bayer encodings
    BAYER_RGGB8 = "bayer_rggb8"
    BAYER_BGGR8 = "bayer_bggr8"
    BAYER_GBRG8 = "bayer_gbrg8"
    BAYER_GRBG8 = "bayer_grbg8"
    BAYER_RGGB16 = "bayer_rggb16"
    BAYER_BGGR16 = "bayer_bggr16"
    BAYER_GBRG16 = "bayer_gbrg16"
    BAYER_GRBG16 = "bayer_grbg16"


ROS_ENCODING_TO_NUMPY = {
    Encoding.RGB8: (np.uint8, 3),
    Encoding.RGBA8: (np.uint8, 4),
    Encoding.RGB16: (np.uint16, 3),
    Encoding.RGBA16: (np.uint16, 4),
    Encoding.BGR8: (np.uint8, 3),
    Encoding.BGRA8: (np.uint8, 4),
    Encoding.BGR16: (np.uint16, 3),
    Encoding.BGRA16: (np.uint16, 4),
    Encoding.MONO8: (np.uint8, 1),
    Encoding.MONO16: (np.uint16, 1),
    Encoding.TYPE_8UC1: (np.uint8, 1),
    Encoding.TYPE_8UC2: (np.uint8, 2),
    Encoding.TYPE_8UC3: (np.uint8, 3),
    Encoding.TYPE_8UC4: (np.uint8, 4),
    Encoding.TYPE_8SC1: (np.int8, 1),
    Encoding.TYPE_8SC2: (np.int8, 2),
    Encoding.TYPE_8SC3: (np.int8, 3),
    Encoding.TYPE_8SC4: (np.int8, 4),
    Encoding.TYPE_16UC1: (np.uint16, 1),
    Encoding.TYPE_16UC2: (np.uint16, 2),
    Encoding.TYPE_16UC3: (np.uint16, 3),
    Encoding.TYPE_16UC4: (np.uint16, 4),
    Encoding.TYPE_16SC1: (np.int16, 1),
    Encoding.TYPE_16SC2: (np.int16, 2),
    Encoding.TYPE_16SC3: (np.int16, 3),
    Encoding.TYPE_16SC4: (np.int16, 4),
    Encoding.TYPE_32SC1: (np.int32, 1),
    Encoding.TYPE_32SC2: (np.int32, 2),
    Encoding.TYPE_32SC3: (np.int32, 3),
    Encoding.TYPE_32SC4: (np.int32, 4),
    Encoding.TYPE_32FC1: (np.float32, 1),
    Encoding.TYPE_32FC2: (np.float32, 2),
    Encoding.TYPE_32FC3: (np.float32, 3),
    Encoding.TYPE_32FC4: (np.float32, 4),
    Encoding.TYPE_64FC1: (np.float64, 1),
    Encoding.TYPE_64FC2: (np.float64, 2),
}


@dataclass
class Image:
    header: Header = field(default_factory=lambda: Header.auto())
    data: np.ndarray = field(default_factory=lambda: np.array([]))
    encoding: Encoding = Encoding.BGR8
    is_bigendian: bool = False
    type: str = "sensor_msgs/Image"

    @classmethod
    def from_other(cls, other: Image) -> Image:
        return Image(
            header=Header(other.header.stamp, other.header.frame_id, other.header.seq),
            data=other.data.copy(),
            encoding=other.encoding,
            is_bigendian=other.is_bigendian,
        )

    def to_compressed(self) -> CompressedImage:
        return CompressedImage(header=self.header, data=self.data)

    @classmethod
    def from_compressed(cls, compressed: CompressedImage) -> Image:
        return Image(
            header=Header(compressed.header.stamp, compressed.header.frame_id, compressed.header.seq),
            data=compressed.data,
            encoding=Encoding.BGR8,
            is_bigendian=False,
        )

    @classmethod
    def from_compressed_depth(cls, compressed: CompressedDepthImage) -> Image:
        return Image(
            header=Header(compressed.header.stamp, compressed.header.frame_id, compressed.header.seq),
            data=compressed.data,
            encoding=Encoding.MONO16,
            is_bigendian=False,
        )

    def to_raw(self) -> RawRosMessage:
        data = base64.b64encode(self.data.tobytes()).decode("ascii")
        return {
            "header": self.header.to_raw(),
            "height": self.data.shape[0],
            "width": self.data.shape[1],
            "encoding": self.encoding.value,
            "is_bigendian": int(self.is_bigendian),
            "step": self.data.shape[1] * self.data.shape[2] * self.data.dtype.itemsize,
            "data": data,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Image:
        encoding = Encoding(msg["encoding"])
        data: str = msg["data"]
        width: int = msg["width"]
        height: int = msg["height"]
        dtype, num_channels = ROS_ENCODING_TO_NUMPY[encoding]

        base64_bytes = data.encode("ascii")
        buffer = np.frombuffer(base64.b64decode(base64_bytes), dtype=dtype)

        if num_channels == 1:
            image = buffer.reshape((height, width))
        else:
            image = buffer.reshape((height, width, num_channels))

        # If the byte order is different between the message and the system.
        is_bigendian = msg["is_bigendian"] != 0
        if is_bigendian == (sys.byteorder == "little"):
            image = image.byteswap().newbyteorder()

        return cls(Header.from_raw(msg["header"]), image, encoding, is_bigendian)
