import os

import cv2
import numpy as np
import torch
import tqdm
from cotracker.predictor import CoTrackerPredictor
from cotracker.utils.visualizer import Visualizer


def load_images():
    directory = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images"

    frames = []
    files = sorted(os.listdir(directory))[100:200]
    pbar = tqdm.tqdm(total=len(files))
    for filename in files:
        pbar.update(1)
        if filename.endswith(".jpg") or filename.endswith(".png"):
            img = cv2.imread(os.path.join(directory, filename))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            frames.append(img)
    pbar.close()
    return frames


def load_video():
    path = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608.mp4"
    cap = cv2.VideoCapture(path)
    frames = []
    start_frame = 5200
    end_frame = start_frame + 50
    pbar = tqdm.tqdm(total=end_frame - start_frame)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    for index in range(start_frame, end_frame):
        pbar.update(1)
        ret, frame = cap.read()
        if not ret:
            break
        show_frame = cv2.resize(frame, (640, 480))
        cv2.imshow("frame", show_frame)
        cv2.waitKey(1)
        if index < start_frame:
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frames.append(frame)
    cap.release()
    pbar.close()
    cv2.imwrite("saved_videos/first_frame.jpg", cv2.cvtColor(frames[0], cv2.COLOR_RGB2BGR))
    return frames


# frames = load_images()
frames = load_video()

device = "cuda"
video = torch.from_numpy(np.stack(frames)).permute(0, 3, 1, 2)[None].float()
video = video.to(device)


query = torch.tensor(
    [
        [0.0, 1167.0, 410.0],
        [0.0, 1242.0, 449.0],
    ]
).to(device)

# Run Offline CoTracker:
print("Running CoTracker...")
model = CoTrackerPredictor(checkpoint=os.path.join("/opt/facebookresearch/co-tracker/checkpoints/scaled_offline.pth"))
model = model.to(device)
pred_tracks, pred_visibility = model(video, queries=query[None])
print("Done!")

vis = Visualizer(save_dir="./saved_videos", pad_value=0, linewidth=3)
breakpoint()
vis.visualize(video, pred_tracks, pred_visibility)
