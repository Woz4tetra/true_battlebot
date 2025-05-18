import gc

import cv2
import numpy as np
import torch
from cotracker.predictor import CoTrackerPredictor
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
    cotracker = CoTrackerPredictor(checkpoint="/home/bwbots/.cache/co-tracker/scaled_offline.pth")
    cotracker = cotracker.to(device)
    query_coordinates = [
        [0, 711.6411894483497, 158.9850087293696],
        [0, 686.7800441243893, 140.54632594743234],
        [0, 598.2737293058256, 215.01613373587767],
        [0, 576.5999103054498, 194.29851263257734],
        [0, 442.33599276935877, 853.776873706832],
        [0, 294.9321903814872, 968.9689188032922],
        [0, 1463.207818441351, 754.5927407814276],
        [0, 1537.6196291496547, 786.9563619132103],
        [0, 1407.78010052413, 933.3764146649078],
        [0, 1460.370984863277, 959.512490518302],
    ]
    query = torch.tensor(np.array(query_coordinates, dtype=np.float32)).to(device)
    video_tensor = torch.from_numpy(np.stack([image])).permute(0, 3, 1, 2)[None].float()
    video_tensor = video_tensor.to(device)
    pred_tracks, pred_visibility = cotracker(video_tensor, queries=query[None])
    del cotracker
    del video_tensor
    gc.collect()

    input_point = np.array(
        [
            [[711.6411743164062, 158.98501586914062], [686.780029296875, 140.54632568359375]],
            [[598.2737426757812, 215.01614379882812], [576.5999145507812, 194.2985076904297]],
            [[442.33599853515625, 853.77685546875], [294.93218994140625, 968.968994140625]],
            [[1463.207763671875, 754.5927124023438], [1537.61962890625, 786.9564208984375]],
            [[1407.7801513671875, 933.3764038085938], [1460.3709716796875, 959.512451171875]],
        ],
        dtype=np.float32,
    )
    input_label = np.array([[1, 1], [1, 1], [1, 1], [1, 1], [1, 1]], dtype=np.int32)

    predictor.set_image(image)
    batch_masks, batch_scores, batch_logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
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
