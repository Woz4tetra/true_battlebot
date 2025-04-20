import logging

from app.config.field_filter.live_pose_field_tracker_config import LivePoseFieldTrackerConfig
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from bw_shared.messages.header import Header
from perception_tools.rosbridge.transform_broadcaster_bridge import TransformBroadcasterBridge


class LivePoseFieldTracker:
    def __init__(self, config: LivePoseFieldTrackerConfig, tf_broadcaster: TransformBroadcasterBridge) -> None:
        self.config = config
        self.tf_worldstart_from_world: Transform3D | None = None
        self.tf_broadcaster = tf_broadcaster
        self.relative_field: EstimatedObject | None = None
        self.logger = logging.getLogger(self.__class__.__name__)

    def reset(self, relative_field: EstimatedObject) -> None:
        self.relative_field = relative_field
        self.tf_worldstart_from_world = None

    def get_tf_worldstart_from_camera(self, tf_camera_from_world: Transform3DStamped) -> Transform3DStamped:
        if self.tf_worldstart_from_world is None and self.relative_field is not None:
            self.tf_worldstart_from_world = tf_camera_from_world.transform
            tf_maprelative_from_worldstart = Transform3DStamped(
                header=Header.from_msg(self.relative_field.header),
                child_frame_id=tf_camera_from_world.child_frame_id,
                transform=Transform3D.from_pose_msg(self.relative_field.pose.pose),
            )
            self.logger.info(f"Publishing static transform {tf_maprelative_from_worldstart}")
            self.tf_broadcaster.publish_static_transforms(tf_maprelative_from_worldstart)

        if self.tf_worldstart_from_world is None:
            tf_worldstart_from_world = Transform3D.identity()
        else:
            tf_worldstart_from_world = self.tf_worldstart_from_world
        tf_worldstart_from_camera = Transform3DStamped(
            header=Header.auto(stamp=tf_camera_from_world.header.stamp, frame_id=tf_camera_from_world.child_frame_id),
            child_frame_id=tf_camera_from_world.header.frame_id,
            transform=tf_worldstart_from_world.forward_by(tf_camera_from_world.transform.inverse()),
        )
        self.tf_broadcaster.publish_transforms(tf_worldstart_from_camera)
        return tf_worldstart_from_camera
