from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TransformBroadcasterBridge:
    """
    A bridge class that wraps a ROS TransformBroadcaster and StaticTransformBroadcaster.
    This class is used to publish transforms in a ROS environment.
    """

    def __init__(self, static_broadcaster: StaticTransformBroadcaster, tf_broadcaster: TransformBroadcaster) -> None:
        self.static_broadcaster = static_broadcaster
        self.tf_broadcaster = tf_broadcaster
        self._has_pending_static_transforms = False
        self._tracked_static_transforms: list[Transform3DStamped] = []
        self._pending_transforms: list[Transform3DStamped] = []

    def publish_static_transforms(self, *transform_stamped: Transform3DStamped) -> None:
        self._tracked_static_transforms.extend(list(transform_stamped))
        self._has_pending_static_transforms = True

    def publish_transforms(self, *transform_stamped: Transform3DStamped) -> None:
        self._pending_transforms.extend(list(transform_stamped))

    def send_pending_transforms(self) -> None:
        if self._pending_transforms:
            self.tf_broadcaster.sendTransform([transform.to_msg() for transform in self._pending_transforms])
            self._pending_transforms.clear()

        if self._has_pending_static_transforms:
            self.static_broadcaster.sendTransform([transform.to_msg() for transform in self._tracked_static_transforms])
            self._has_pending_static_transforms = False
