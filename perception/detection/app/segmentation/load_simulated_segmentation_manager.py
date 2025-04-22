from app.config.config import Config
from app.container import Container
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager
from bw_interfaces.msg import SegmentationInstanceArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


def load_simulated_segmentation_manager(container: Container) -> SimulatedSegmentationManager:
    if container.is_registered(SimulatedSegmentationManager):
        return container.resolve(SimulatedSegmentationManager)
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    sim_segmentation_image_sub = RosPollSubscriber(namespace + "/layer/image_raw", Image, buff_size=2 << 24)
    simulated_segmentation_sub = RosPollSubscriber(namespace + "/simulated_segmentation", SegmentationInstanceArray)
    layer_request_pub = RosPublisher(namespace + "/layer/request", Empty)

    manager = SimulatedSegmentationManager(sim_segmentation_image_sub, simulated_segmentation_sub, layer_request_pub)
    container.register(manager)
    return manager
