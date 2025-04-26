import os

import cv2
import imageio.v3 as iio
import torch
import tqdm
from cotracker.utils.visualizer import Visualizer

# Download the video
# url = "https://github.com/facebookresearch/co-tracker/raw/refs/heads/main/assets/apple.mp4"
directory = "/data/videos/Cage-2-Overhead-High-2024-10-26_15-30-33.608_trim_images"


# frames = iio.imread(url, plugin="FFMPEG")  # plugin="pyav"

frames = []
files = sorted(os.listdir(directory))[100:120]
pbar = tqdm.tqdm(total=len(files))
for filename in files:
    pbar.update(1)
    if filename.endswith(".jpg") or filename.endswith(".png"):
        img = cv2.imread(os.path.join(directory, filename))
        frames.append(img)

device = "cuda"
grid_size = 50
video = torch.tensor(frames).permute(0, 3, 1, 2)[None].float().to(device)  # B T C H W

# Run Offline CoTracker:
print("Running CoTracker...")
cotracker = torch.hub.load("facebookresearch/co-tracker", "cotracker3_offline").to(device)
pred_tracks, pred_visibility = cotracker(video, grid_size=grid_size)
print("Done!")

vis = Visualizer(save_dir="./saved_videos", pad_value=120, linewidth=3)
vis.visualize(video, pred_tracks, pred_visibility)
