from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber


class SimulatedSegmentationManager:
    def __init__(
        self,
        sim_segmentation_image_sub: RosPollSubscriber[CompressedImage],
        simulated_segmentation_sub: RosPollSubscriber[SegmentationInstanceArray],
    ) -> None:
        self.sim_segmentation_image_sub = sim_segmentation_image_sub
        self.simulated_segmentation_sub = simulated_segmentation_sub
        self.prev_image: CompressedImage | None = None
        self.prev_segmentation: SegmentationInstanceArray | None = None

    def get_image(self) -> CompressedImage | None:
        if image := self.sim_segmentation_image_sub.receive():
            self.prev_image = image
        return self.prev_image

    def get_segmentation(self) -> SegmentationInstanceArray | None:
        if segmentation := self.simulated_segmentation_sub.receive():
            self.prev_segmentation = segmentation
        return self.prev_segmentation
