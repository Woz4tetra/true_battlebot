import logging

from app.config.field_filter.field_request_config import FieldRequestConfig
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from std_msgs.msg import Empty


class FieldRequestHandler:
    def __init__(
        self,
        field_request_config: FieldRequestConfig,
        request_subscriber: RosPollSubscriber[Empty],
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.stale_image_timeout = field_request_config.stale_image_timeout
        self.request_subscriber = request_subscriber

    def has_request(self) -> bool:
        if self.request_subscriber.receive():
            self.logger.info("Received field request")
            return True
        return False
