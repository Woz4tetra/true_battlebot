import pytest
from app.config.segmentation_config.semantic_segmentation_config import SemanticSegmentationConfig
from app.segmentation.semantic_segmentation import SemanticSegmentation
from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap
from bw_shared.enums.label import Label, ModelLabel


@pytest.fixture(scope="module")
def semantic_segmentation_config() -> SemanticSegmentationConfig:
    return SemanticSegmentationConfig(
        model_path="field_deeplabv3_r50_2024-09-05.torchscript",
        metadata_path="field_deeplabv3_r50_2024-09-05.json",
        debug=True,
        model_to_system_labels=ModelToSystemLabelsMap(
            {ModelLabel.FIELD: Label.FIELD, ModelLabel.BACKGROUND: Label.BACKGROUND}
        ),
    )


@pytest.fixture(scope="module")
def semantic_segmentation(semantic_segmentation_config: SemanticSegmentationConfig) -> SemanticSegmentation:
    return SemanticSegmentation(semantic_segmentation_config)
