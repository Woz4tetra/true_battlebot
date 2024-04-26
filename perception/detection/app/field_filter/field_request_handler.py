import logging
import time

from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.empty import Empty
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher

from app.config.field_filter_config.field_request_config import FieldRequestConfig


class FieldRequestHandler:
    def __init__(
        self,
        field_request_config: FieldRequestConfig,
        request_subscriber: RosPollSubscriber[Empty],
        response_publisher: RosPublisher[FieldResult],
    ) -> None:
        self.logger = logging.getLogger("perception")
        self.stale_image_timeout = field_request_config.stale_image_timeout
        self.request_subscriber = request_subscriber
        self.response_publisher = response_publisher

    def has_request(self, last_image_time: float) -> bool:
        received = self.request_subscriber.receive()
        if received is not None:
            self.logger.info("Received field request")
            now = time.time()
            delay = now - last_image_time
            if now - last_image_time > self.stale_image_timeout:
                self.logger.warning(f"Image is {delay:0.4f} seconds stale. Dropping request.")
                return False
            return True
        return False

    def send_response(self, field_result: FieldResult) -> None:
        self.response_publisher.publish(field_result)
