import cv2
import numpy as np
import torch
from matplotlib import pyplot as plt
from segment_anything import SamAutomaticMaskGenerator, SamPredictor, sam_model_registry
from supervision import Detections, MaskAnnotator

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

CHECKPOINT = "sam_vit_h_4b8939.pth"
MODEL_TYPE = "default"
image = cv2.imread("Screenshot from 2023-09-23 10-57-03.png")


def segment_point(image, point):
    sam = sam_model_registry[MODEL_TYPE](checkpoint=CHECKPOINT)
    predictor = SamPredictor(sam)
    predictor.set_image(image)
    results = predictor.predict(np.array([point]), np.array([1]))
    return results


def segment_anything(image):
    print("Segmenting image...")
    sam = sam_model_registry[MODEL_TYPE](checkpoint=CHECKPOINT)
    print("Loaded SAM model.")
    mask_generator = SamAutomaticMaskGenerator(sam)
    masks = mask_generator.generate(image)
    print("Done.")
    return masks


def draw_masks(image, masks):
    mask_annotator = MaskAnnotator()
    detections = Detections.from_sam(masks)
    annotated_image = mask_annotator.annotate(image, detections)
    return annotated_image


masks = segment_anything(image)
annotated_image = draw_masks(image, masks)
breakpoint()
plt.imshow(annotated_image)
plt.show()
