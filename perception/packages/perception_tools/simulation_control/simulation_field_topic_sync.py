import logging
from dataclasses import dataclass
from threading import Lock

import rospy
from bw_interfaces.msg import EstimatedObject
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg

from perception_tools.messages.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber


@dataclass
class FieldDataShapshot:
    model: PinholeCameraModel
    image: Image
    field: EstimatedObject


class SimulationFieldTopicSync:
    def __init__(self) -> None:
        self.logger = logging.getLogger("perception")
        self.model: PinholeCameraModel | None = None
        self.image: Image | None = None
        self.field: EstimatedObject | None = None
        self.lock = Lock()

        camera_ns = "/camera_0/"
        self.simulated_field_sub = rospy.Subscriber(
            camera_ns + "simulated_field_result", EstimatedObject, self.field_callback, queue_size=1
        )
        self.image_sub = RosPollSubscriber(camera_ns + "rgb/image_raw", ImageMsg)
        self.camera_info_sub = RosPollSubscriber(camera_ns + "rgb/camera_info", CameraInfo)

        self.cache_size = 100
        self.image_max_delay = 0.01
        self.field_cache: dict[float, EstimatedObject] = {}
        self.lock = Lock()

    def field_callback(self, msg: EstimatedObject) -> None:
        with self.lock:
            if not self.model and (cam_info := self.camera_info_sub.receive()):
                self.model = PinholeCameraModel()
                self.model.fromCameraInfo(cam_info)

            if img_msg := self.image_sub.receive():
                self.image = Image.from_msg(img_msg)

            self.field_cache[msg.header.stamp.to_sec()] = msg
            if self.image is None:
                return

            image_timestamp = self.image.header.stamp
            selected_key = min(self.field_cache.keys(), key=lambda k: abs(k - image_timestamp))
            selected_msg = self.field_cache[selected_key]
            if abs(selected_key - self.image.header.stamp) <= self.image_max_delay:
                self.field = selected_msg
            else:
                self.logger.info(f"Field and image timestamps are too far apart. {selected_key - image_timestamp}")
                self.field = None
            while len(self.field_cache) > self.cache_size:
                del self.field_cache[min(self.field_cache.keys())]

    def get_snapshot(self) -> FieldDataShapshot | None:
        with self.lock:
            if self.model is None:
                self.logger.warning("Missing camera model")
                return None

            if self.image is None:
                self.logger.warning("Missing image")
                return None

            if self.field is None:
                self.logger.warning("Missing field")
                return None

            image = self.image
            self.image = None
            field = self.field
            self.field = None

            return FieldDataShapshot(self.model, image, field)
