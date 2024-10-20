import cv2
import pytest
from app.keypoint.yolo_keypoint import YoloKeypoint
from bw_shared.messages.header import Header
from perception_tools.data_directory import get_data_directory
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


@pytest.fixture
def camera_info() -> CameraInfo:
    return CameraInfo()


TEST_IMAGES = (
    "1725200609.012655735.jpg",
    "1721512481-7671976-000300_jpg.rf.224f13d55aa5a5076f31f53c8fdc9472.jpg",
    "Cage-5-Overhead-High_1080p_174-000750_jpg.rf.cd7457e0cdd4f501fd5fed1cc9e49783.jpg",
)


def load_image(image_name: str) -> Image:
    data_dir = get_data_directory()
    image_path = data_dir / "images" / "yolo_keypoint" / image_name
    assert image_path.is_file()
    image = cv2.imread(str(image_path))
    return Image(header=Header.auto(frame_id="camera_0"), data=image)


@pytest.mark.parametrize("image_path", TEST_IMAGES)
def test_yolo_keypoint(image_path: str, yolo_keypoint: YoloKeypoint, camera_info: CameraInfo):
    image = load_image(image_path)
    result, debug_msg = yolo_keypoint.process_image(camera_info, image)
    assert result is not None and len(result.instances) >= 1
