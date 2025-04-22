from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from bw_shared.messages.field import Field


class FieldTransformManager:
    def __init__(self) -> None:
        self.field: Field | None = None
        self.tf_worldstart_from_camera: Transform3DStamped | None = None

    def update_field(self, msg: EstimatedObject) -> None:
        self.field = Field.from_msg(msg)
        self.tf_worldstart_from_camera = None

    def update_camera_transform(self, camera_transform: Transform3DStamped) -> None:
        self.tf_worldstart_from_camera = camera_transform

    def get_field(self) -> Field | None:
        if self.tf_worldstart_from_camera is None or self.field is None:
            return None
        tf_map_from_camerastart = self.field.tf_map_from_camera
        tf_map_from_camera = tf_map_from_camerastart.transform_by(self.tf_worldstart_from_camera.transform)
        return Field(
            header=self.field.header,
            child_frame_id=self.tf_worldstart_from_camera.child_frame_id,
            tf_map_from_camera=tf_map_from_camera,
            size=self.field.size,
            label=self.field.label,
        )
