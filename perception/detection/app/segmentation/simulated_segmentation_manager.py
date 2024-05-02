from bw_interfaces.msg import SegmentationInstanceArray
from perception_tools.messages.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import Image as RosImage


class SimulatedSegmentationManager:
    def __init__(
        self,
        sim_segmentation_image_sub: RosPollSubscriber[RosImage],
        simulated_segmentation_sub: RosPollSubscriber[SegmentationInstanceArray],
    ) -> None:
        self.sim_segmentation_image_sub = sim_segmentation_image_sub
        self.simulated_segmentation_sub = simulated_segmentation_sub
        self.prev_image: Image | None = None
        self.prev_segmentation: SegmentationInstanceArray | None = None

    def get_image(self) -> Image | None:
        if image := self.sim_segmentation_image_sub.receive():
            self.prev_image = Image.from_msg(image)
        return self.prev_image

    def get_segmentation(self) -> SegmentationInstanceArray | None:
        if segmentation := self.simulated_segmentation_sub.receive():
            self.prev_segmentation = segmentation
        return self.prev_segmentation
