import logging

from app.config.field_filter.field_request_config import FieldRequestConfig
from bw_interfaces.msg import EstimatedObject
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from std_msgs.msg import Empty


class FieldRequestHandler:
    def __init__(
        self,
        field_request_config: FieldRequestConfig,
        request_subscriber: RosPollSubscriber[Empty],
        response_publisher: RosPublisher[EstimatedObject],
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.stale_image_timeout = field_request_config.stale_image_timeout
        self.request_subscriber = request_subscriber
        self.response_publisher = response_publisher

    def has_request(self) -> bool:
        if self.request_subscriber.receive():
            self.logger.info("Received field request")
            return True
        return False

    def send_response(self, field_result: EstimatedObject) -> None:
        self.response_publisher.publish(field_result)
