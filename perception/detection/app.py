from container import Container
from perception_tools.rosbridge.ros_factory import RosFactory

from perception.detection.camera.camera_interface import CameraInterface
from perception.detection.field_filter.field_filter import FieldFilter
from perception.detection.field_filter.field_request_handler import FieldRequestHandler


class App:
    def __init__(self, container: Container) -> None:
        self.container = container
        self.ros_factory = self.container.resolve(RosFactory)
        self.camera = self.container.resolve(CameraInterface)
        self.field_segmentation = self.container.resolve("field_segmentation")
        self.field_filter = self.container.resolve(FieldFilter)
        self.field_request_handler = self.container.resolve(FieldRequestHandler)
        self.field_debug_image_publisher = self.container.resolve("field_debug_image_publisher")
        self.camera_data = None

    def start(self) -> None:
        self.ros_factory.connect()

    def loop(self) -> None:
        camera = self.camera
        field_segmentation = self.field_segmentation
        field_filter = self.field_filter
        field_request_handler = self.field_request_handler

        if camera_data := camera.poll():
            self.camera_data = camera_data
        if self.camera_data is not None and field_request_handler.has_request(
            self.camera_data.camera_info.header.stamp
        ):
            image = self.camera_data.color_image
            seg_result, debug_image = field_segmentation.process_image(image)
            field_result = field_filter.compute_field(
                seg_result, self.camera_data.depth_image, self.camera_data.camera_info
            )
            field_request_handler.send_response(field_result)
            if debug_image:
                self.field_debug_image_publisher.publish(debug_image.to_ros_image())

    def stop(self) -> None:
        self.ros_factory.disconnect()
