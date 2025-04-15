from __future__ import annotations

import logging
from dataclasses import dataclass, field

from app.config.camera.camera_types import CameraConfig
from app.config.camera.noop_camera_config import NoopCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from app.config.field_filter.field_filter_types import FieldFilterConfig
from app.config.field_filter.field_request_config import FieldRequestConfig
from app.config.field_filter.point_cloud_field_filter_config import PointCloudFieldFilterConfig
from app.config.keypoint_config.keypoint_types import KeypointConfig, YoloKeypointConfig
from app.config.ros_config import RosConfig
from app.config.segmentation.segmentation_types import SegmentationConfig, SemanticSegmentationConfig
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class Config:
    log_level: int = logging.DEBUG
    target_tick_rate: float = 1000.0
    loop_overrun_threshold: float = 0.2  # seconds
    camera_topic: CameraTopicConfig = field(default_factory=lambda: CameraTopicConfig("/camera_0", "camera_0"))
    camera: CameraConfig = field(default_factory=NoopCameraConfig)
    ros: RosConfig = field(default_factory=RosConfig)
    field_segmentation: SegmentationConfig = field(default_factory=SemanticSegmentationConfig)
    robot_keypoint: KeypointConfig = field(default_factory=YoloKeypointConfig)
    field_filter: FieldFilterConfig = field(default_factory=PointCloudFieldFilterConfig)
    field_request: FieldRequestConfig = field(default_factory=FieldRequestConfig)

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
