from __future__ import annotations

import base64
from dataclasses import dataclass, field

import cv2
import numpy as np
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage

PNG_HEADER = b"\x89PNG\r\n\x1a\n"


@dataclass
class CompressedDepthImage:
    header: Header = field(default_factory=lambda: Header.auto())
    format: str = "16UC1; compressedDepth png"
    data: np.ndarray = field(default_factory=lambda: np.array([]))
    type: str = "sensor_msgs/CompressedImage"

    @classmethod
    def from_other(cls, other: CompressedDepthImage) -> CompressedDepthImage:
        return CompressedDepthImage(
            header=Header(other.header.stamp, other.header.frame_id, other.header.seq),
            data=other.data.copy(),
            format=other.format,
        )

    def to_raw(self) -> RawRosMessage:
        extension = ".png"
        if len(self.data) != 0:
            success, encoded_data = cv2.imencode(extension, self.data, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not success:
                raise ValueError(f"Failed to encode image. {self}")
            data = base64.b64encode(encoded_data.tobytes()).decode("ascii")
        else:
            data = ""
        return {
            "header": self.header.to_raw(),
            "format": self.format,
            "data": data,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> CompressedDepthImage:
        data: str = msg["data"]
        format = msg["format"]
        data_bytes = base64.b64decode(data.encode("ascii"))
        data_bytes = data_bytes[data_bytes.index(PNG_HEADER) :]
        encoded_array = np.frombuffer(data_bytes, np.uint8)
        image = cv2.imdecode(encoded_array, cv2.IMREAD_ANYDEPTH)
        if image is None:
            msg_str = str(msg)
            if len(msg_str) > 200:
                msg_str = msg_str[0:200] + "..."
            raise ValueError(f"Failed to decode image. {msg_str}")

        return cls(Header.from_raw(msg["header"]), format, image)
