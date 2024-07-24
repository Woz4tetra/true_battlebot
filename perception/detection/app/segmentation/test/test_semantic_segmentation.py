import cv2
import pytest
from app.config.segmentation_config.semantic_segmentation_config import SemanticSegmentationConfig
from app.segmentation.semantic_segmentation import SemanticSegmentation
from perception_tools.data_directory import get_data_directory
from perception_tools.messages.image import Image


@pytest.fixture
def semantic_segmentation_config() -> SemanticSegmentationConfig:
    return SemanticSegmentationConfig(
        model_path="field_deeplabv3_mbv3_2024-06-17.torchscript",
        metadata_path="field_deeplabv3_mbv3_2024-06-17.json",
        debug=True,
    )


@pytest.fixture
def semantic_segmentation(semantic_segmentation_config: SemanticSegmentationConfig) -> SemanticSegmentation:
    return SemanticSegmentation(semantic_segmentation_config)


TEST_IMAGES = (
    "20230506_115229.jpg",
    "3lb-48-silentspring-vs-narcissist-4-001200_jpg.rf.25d85dc01690aff4c98eb1b9f95ccb06_augment-000.jpg",
    "Cage-3-Overhead-High_1080p_86-000250_jpg.rf.77f42dddcb242860e8f3b40156b34d35_augment-000.jpg",
    "Cage-3-Overhead-High_1080p_86-000250_jpg.rf.77f42dddcb242860e8f3b40156b34d35.jpg",
    "mini_bot_2024-03-02T09-46-00-000950_jpg.rf.2809d85a9d26623494a7d4a32a6ae614.jpg",
    "mini_bot_2024-03-02T16-20-33-004300_jpg.rf.4e92a1c019d118da87067d88c0652000.jpg",
    "zed_2023-09-30T17-01-47_repaired_001904_jpg.rf.04d83ab60612bb3a89d8118f22007eda.jpg",
)


def load_image(image_name: str) -> Image:
    data_dir = get_data_directory()
    image_path = data_dir / "images" / "semantic_segmentation" / image_name
    assert image_path.is_file()
    image = cv2.imread(str(image_path))
    return Image(data=image)


@pytest.mark.parametrize("image_path", TEST_IMAGES)
def test_semantic_segmentation(image_path: str, semantic_segmentation: SemanticSegmentation):
    image = load_image(image_path)
    result, debug_msg = semantic_segmentation.process_image(image)
    assert len(result.instances) == 1
