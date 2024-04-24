import logging
import time

from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.ros_poll_subscriber import RosPollRawSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher

from app.config.field_filter_config.field_request_config import FieldRequestConfig


class FieldRequestHandler:
    def __init__(
        self,
        field_request_config: FieldRequestConfig,
        request_subscriber: RosPollRawSubscriber,
        response_publisher: RosPublisher[FieldResult],
    ) -> None:
        self.logger = logging.getLogger("perception")
        self.stale_image_timeout = field_request_config.stale_image_timeout
        self.request_subscriber = request_subscriber
        self.response_publisher = response_publisher

    def has_request(self, last_image_time: float) -> bool:
        now = time.time()
        if now - last_image_time > self.stale_image_timeout:
            self.logger.debug("Image is stale. Dropping request.")
            return False
        return self.request_subscriber.receive() is not None

    def send_response(self, field_result: FieldResult) -> None:
        self.response_publisher.publish(field_result)
