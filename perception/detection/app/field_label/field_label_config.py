from dataclasses import dataclass


@dataclass
class FieldLabelConfig:
    cloud_topic: str = "/camera_0/point_cloud/cloud_registered"
    image_topic: str = "/camera_0/rgb/image_raw"
    info_topic: str = "/camera_0/rgb/camera_info"
    field_request_topic: str = "/perception/field/request"
    field_response_topic: str = "/perception/field/response"
    max_cloud_distance: float = 1000.0
    label_state_path: str = "/media/storage/label_state.json"
