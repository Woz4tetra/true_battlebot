import logging
import time

from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.ros_poll_subscriber import RosPollRawSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher


class FieldRequestHandler:
    def __init__(
        self,
        stale_image: float,
        request_subscriber: RosPollRawSubscriber,
        response_subscriber: RosPublisher[FieldResult],
    ) -> None:
        self.logger = logging.getLogger("perception")
        self.stale_image = stale_image
        self.request_subscriber = request_subscriber
        self.response_subscriber = response_subscriber

    def has_request(self, last_image_time: float) -> bool:
        now = time.time()
        if now - last_image_time > self.stale_image:
            self.logger.debug("Image is stale. Dropping request.")
            return False
        return self.request_subscriber.receive() is not None

    def send_response(self, field_result: FieldResult) -> None:
        self.response_subscriber.publish(field_result)
