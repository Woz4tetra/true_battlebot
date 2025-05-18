import cv2
import numpy as np
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# select the device for computation
device = torch.device("cuda")
print(f"using device: {device}")

# use bfloat16 for the entire notebook
torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
# turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
if torch.cuda.get_device_properties(0).major >= 8:
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
# np.random.seed(3)


def show_mask(mask, ax, random_color=False, borders=True):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])
    h, w = mask.shape[-2:]
    mask = mask.astype(np.uint8)
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    if borders:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Try to smooth contours
        contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
        mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2)
    ax.imshow(mask_image)


def show_masks(image, masks):
    combined_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    height, width = image.shape[:2]
    for mask in masks:
        mask = (mask > 0.0).astype(np.uint8)
        cv2.imshow("mask", mask.reshape(height, width, 1) * 255)
        cv2.waitKey(-1)
        combined_mask = np.logical_or(combined_mask, mask)
    mask_image = np.zeros(image.shape, dtype=np.uint8)
    mask_image[combined_mask] = image[combined_mask]
    cv2.imshow("mask", mask_image)
    cv2.waitKey(-1)


def main() -> None:
    # image = Image.open("images/nhrl.jpg")
    # image = np.array(image.convert("RGB"))
    image = cv2.imread("images/nhrl.jpg")

    sam2_checkpoint = "/home/bwbots/.cache/sam2/sam2.1_hiera_large.pt"
    model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"

    sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)

    predictor = SAM2ImagePredictor(sam2_model)

    input_box = np.array(
        [(687, 138, 28, 20), (576, 194, 23, 22), (294, 853, 149, 116), (1463, 754, 75, 33), (1407, 933, 54, 27)],
        dtype=np.float32,
    )

    predictor.set_image(image)
    batch_masks, batch_scores, batch_logits = predictor.predict(
        box=input_box,
        multimask_output=True,
    )
    all_masks = []
    for index in range(len(batch_masks)):
        masks = batch_masks[index]
        scores = batch_scores[index]
        logits = batch_logits[index]
        max_index = np.argmax(scores)
        mask = masks[max_index]
        score = scores[max_index]
        all_masks.append(mask)

    show_masks(image, all_masks)


main()
