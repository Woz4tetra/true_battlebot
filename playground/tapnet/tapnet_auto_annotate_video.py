#!/usr/bin/env python
# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Auto-annotate video with TAPIR point tracking using automatic foreground point detection."""

import argparse
from typing import List, Tuple

import cv2
import mediapy as media
import numpy as np
import tqdm

from tapnet.models import tapir_model
from tapnet.utils import model_utils
from tapnet.utils import transforms
from tapnet.utils import viz_utils


def load_checkpoint(checkpoint_path):
    """Load TAPIR checkpoint."""
    ckpt_state = np.load(checkpoint_path, allow_pickle=True).item()
    return ckpt_state["params"], ckpt_state["state"]


def detect_foreground_points(
    frames: np.ndarray,
    num_points: int = 50,
    quality_level: float = 0.01,
    min_distance: int = 20,
    use_motion: bool = True,
) -> List[Tuple[int, int, int]]:
    """
    Automatically detect foreground points to track using multiple methods.

    Args:
        frames: Video frames [num_frames, height, width, 3]
        num_points: Maximum number of points to detect
        quality_level: Quality level for corner detection
        min_distance: Minimum distance between detected points
        use_motion: Whether to use motion detection to focus on moving objects

    Returns:
        List of (frame_idx, y, x) tuples representing detected points
    """
    print("Detecting foreground points...")

    # Convert to grayscale
    gray_frames = [
        cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) for frame in frames[: min(10, len(frames))]
    ]

    detected_points = []

    if use_motion and len(gray_frames) > 1:
        # Method 1: Motion-based detection
        print("  Using motion-based detection...")
        motion_points = detect_motion_points(
            gray_frames, num_points // 2, quality_level, min_distance
        )
        detected_points.extend(motion_points)

    # Method 2: Corner detection on first frame
    print("  Using corner detection...")
    corner_points = detect_corner_points(
        gray_frames[0],
        num_points - len(detected_points),
        quality_level,
        min_distance,
        detected_points,
    )
    detected_points.extend(corner_points)

    # Method 3: Edge-based detection if we still need more points
    if len(detected_points) < num_points:
        print("  Using edge-based detection...")
        edge_points = detect_edge_points(
            gray_frames[0],
            num_points - len(detected_points),
            min_distance,
            detected_points,
        )
        detected_points.extend(edge_points)

    # If still not enough, use grid sampling
    if len(detected_points) < num_points:
        print("  Using grid sampling...")
        grid_points = sample_grid_points(
            gray_frames[0], num_points - len(detected_points), detected_points
        )
        detected_points.extend(grid_points)

    print(f"  Detected {len(detected_points)} points")
    return detected_points[:num_points]


def detect_motion_points(
    gray_frames: List[np.ndarray],
    num_points: int,
    quality_level: float,
    min_distance: int,
) -> List[Tuple[int, int, int]]:
    """Detect points in areas with motion between frames."""
    points = []

    for i in range(1, min(len(gray_frames), 5)):  # Check first few frames for motion
        # Compute optical flow
        flow = cv2.calcOpticalFlowPyrLK(
            gray_frames[0], gray_frames[i], None, None, winSize=(15, 15), maxLevel=2
        )

        # Create motion mask
        if flow[0] is not None:
            motion_mag = np.sqrt(flow[0][:, :, 0] ** 2 + flow[0][:, :, 1] ** 2)
        else:
            motion_mag = np.zeros_like(gray_frames[0])
        motion_mask = (motion_mag > 2).astype(np.uint8) * 255

        # Detect corners in motion areas
        corners = cv2.goodFeaturesToTrack(
            gray_frames[0],
            maxCorners=num_points // (i + 1),
            qualityLevel=quality_level,
            minDistance=min_distance,
            mask=motion_mask,
        )

        if corners is not None:
            for corner in corners:
                x, y = corner.ravel().astype(int)
                points.append((0, y, x))  # Frame 0, y, x format

        if len(points) >= num_points:
            break

    return points[:num_points]


def detect_corner_points(
    gray_frame: np.ndarray,
    num_points: int,
    quality_level: float,
    min_distance: int,
    existing_points: List = None,
) -> List[Tuple[int, int, int]]:
    """Detect corner points using goodFeaturesToTrack."""
    if existing_points is None:
        existing_points = []

    # Create mask to avoid existing points
    mask = np.ones_like(gray_frame, dtype=np.uint8) * 255
    for _, y, x in existing_points:
        cv2.circle(mask, (x, y), min_distance, 0, -1)

    corners = cv2.goodFeaturesToTrack(
        gray_frame,
        maxCorners=num_points,
        qualityLevel=quality_level,
        minDistance=min_distance,
        mask=mask,
    )

    points = []
    if corners is not None:
        for corner in corners:
            x, y = corner.ravel().astype(int)
            points.append((0, y, x))  # Frame 0, y, x format

    return points


def detect_edge_points(
    gray_frame: np.ndarray,
    num_points: int,
    min_distance: int,
    existing_points: List = None,
) -> List[Tuple[int, int, int]]:
    """Detect points along strong edges."""
    if existing_points is None:
        existing_points = []

    # Create mask to avoid existing points
    mask = np.ones_like(gray_frame, dtype=np.uint8) * 255
    for _, y, x in existing_points:
        cv2.circle(mask, (x, y), min_distance, 0, -1)

    # Detect edges
    edges = cv2.Canny(gray_frame, 50, 150)
    edges = cv2.bitwise_and(edges, mask)

    # Find edge points
    edge_points = np.where(edges > 0)

    points = []
    if len(edge_points[0]) > 0:
        # Sample points from edges
        indices = np.random.choice(
            len(edge_points[0]), min(num_points, len(edge_points[0])), replace=False
        )
        for idx in indices:
            y, x = edge_points[0][idx], edge_points[1][idx]
            # Check minimum distance from existing points
            too_close = False
            for _, ey, ex in existing_points + points:
                if np.sqrt((y - ey) ** 2 + (x - ex) ** 2) < min_distance:
                    too_close = True
                    break
            if not too_close:
                points.append((0, y, x))
                if len(points) >= num_points:
                    break

    return points


def sample_grid_points(
    gray_frame: np.ndarray, num_points: int, existing_points: List = None
) -> List[Tuple[int, int, int]]:
    """Sample points in a grid pattern, avoiding existing points."""
    if existing_points is None:
        existing_points = []

    h, w = gray_frame.shape
    points = []

    # Calculate grid spacing
    grid_size = int(np.sqrt(num_points)) + 1
    step_y, step_x = h // grid_size, w // grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            y = step_y * i + step_y // 2
            x = step_x * j + step_x // 2

            if y >= h or x >= w:
                continue

            # Check minimum distance from existing points
            too_close = False
            for _, ey, ex in existing_points + points:
                if np.sqrt((y - ey) ** 2 + (x - ex) ** 2) < 20:
                    too_close = True
                    break

            if not too_close:
                points.append((0, y, x))

            if len(points) >= num_points:
                return points

    return points


def batch_inference(
    tapir, frames: np.ndarray, query_points: np.ndarray, chunk_size: int = 32
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Perform batch inference on the entire video using efficient chunking.

    Args:
        tapir: TAPIR model
        frames: Preprocessed video frames
        query_points: Query points in format [num_points, 3] (t, y, x)
        chunk_size: Size of chunks for processing

    Returns:
        tracks: [num_points, num_frames, 2] track positions
        visibles: [num_points, num_frames] visibility flags
    """
    print("Running batch inference...")

    # Precompute feature grids for the entire video
    print("  Computing feature grids...")
    feature_grids = tapir.get_feature_grids(frames, is_training=False)

    # Process points in chunks to manage memory
    all_tracks = []
    all_visibles = []

    num_points = query_points.shape[0]
    print(f"  Processing {num_points} points in chunks of {chunk_size}...")

    for chunk_start in tqdm.tqdm(
        range(0, num_points, chunk_size), desc="Processing point chunks"
    ):
        chunk_end = min(chunk_start + chunk_size, num_points)
        chunk_points = query_points[chunk_start:chunk_end]

        # Run inference for this chunk
        chunk_points_batch = chunk_points.astype(np.float32)[None]  # Add batch dimension

        outputs = tapir(
            video=frames,
            query_points=chunk_points_batch,
            is_training=False,
            query_chunk_size=chunk_size,
            feature_grids=feature_grids,
        )

        tracks = outputs["tracks"][0]  # Remove batch dimension
        occlusions = outputs["occlusion"][0]
        expected_dist = outputs["expected_dist"][0]

        # Binarize occlusions
        visibles = model_utils.postprocess_occlusions(occlusions, expected_dist)

        all_tracks.append(tracks)
        all_visibles.append(visibles)

    # Concatenate all chunks
    tracks = np.concatenate(all_tracks, axis=0)
    visibles = np.concatenate(all_visibles, axis=0)

    return tracks, visibles


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Auto-annotate video with TAPIR point tracking"
    )
    parser.add_argument(
        "video_path",
        type=str,
        help="Path to the input video file.",
    )
    parser.add_argument(
        "-o",
        "--output_path",
        type=str,
        default=None,
        help="Path to the output video file. If not specified, will use input path with '_auto_tracked' suffix.",
    )
    parser.add_argument(
        "-ch",
        "--checkpoint_path",
        type=str,
        default="/opt/deepmind/tapnet/checkpoints/bootstapir_checkpoint_v2.npy",
        help="Path to the TAPIR checkpoint file.",
    )
    parser.add_argument(
        "-s",
        "--start-time",
        type=float,
        default=0.0,
        help="Start time in seconds to begin processing the video.",
    )
    parser.add_argument(
        "-e",
        "--end-time",
        type=float,
        default=None,
        help="End time in seconds to stop processing the video.",
    )
    parser.add_argument(
        "-sz",
        "--size",
        type=int,
        default=256,
        help="Size to which the input video frames will be resized (square).",
    )
    parser.add_argument(
        "-n",
        "--num-points",
        type=int,
        default=50,
        help="Maximum number of points to track automatically.",
    )
    parser.add_argument(
        "-cs",
        "--chunk-size",
        type=int,
        default=32,
        help="Chunk size for batch processing points.",
    )
    parser.add_argument(
        "--model-type",
        type=str,
        default="bootstapir",
        choices=["tapir", "bootstapir"],
        help="Type of TAPIR model to use.",
    )
    parser.add_argument(
        "--no-motion",
        action="store_true",
        help="Disable motion-based point detection.",
    )

    args = parser.parse_args()

    video_path = args.video_path
    checkpoint_path = args.checkpoint_path
    start_time = args.start_time
    end_time = args.end_time
    resize_size = args.size
    num_points = args.num_points
    chunk_size = args.chunk_size
    model_type = args.model_type
    use_motion = not args.no_motion

    # Generate output path if not specified
    if args.output_path is None:
        video_name = video_path.rsplit(".", 1)[0]
        video_ext = video_path.rsplit(".", 1)[1] if "." in video_path else "mp4"
        output_path = f"{video_name}_auto_tracked.{video_ext}"
    else:
        output_path = args.output_path

    print(f"Input video: {video_path}")
    print(f"Output video: {output_path}")
    print(f"Model type: {model_type}")
    print(f"Processing size: {resize_size}x{resize_size}")
    print(f"Max points to track: {num_points}")

    # Load and preprocess video
    print("Loading video...")
    video = media.read_video(video_path)

    # Apply time range if specified
    fps = 30  # Default FPS, could be extracted from video metadata
    if start_time > 0 or end_time is not None:
        start_frame = int(start_time * fps)
        end_frame = int(end_time * fps) if end_time is not None else len(video)
        video = video[start_frame:end_frame]
        print(f"Using frames {start_frame} to {end_frame}")

    print(f"Video shape: {video.shape}")

    # Resize video for processing
    if video.shape[1] != resize_size or video.shape[2] != resize_size:
        print(f"Resizing video to {resize_size}x{resize_size}...")
        original_video = video.copy()
        video = media.resize_video(video, (resize_size, resize_size))
    else:
        original_video = video.copy()

    # Detect foreground points automatically
    detected_points = detect_foreground_points(
        video[: min(10, len(video))],  # Use first 10 frames for detection
        num_points=num_points,
        use_motion=use_motion,
    )

    if not detected_points:
        print("No points detected. Exiting.")
        return

    print(f"Tracking {len(detected_points)} detected points")

    # Convert detected points to query points format
    query_points = np.array([[t, y, x] for t, y, x in detected_points], dtype=np.float32)

    # Load checkpoint and initialize model
    print("Loading checkpoint...")
    params, state = load_checkpoint(checkpoint_path)

    # Configure model kwargs based on model type
    kwargs = dict(bilinear_interp_with_depthwise_conv=False, pyramid_level=0)
    if model_type == "bootstapir":
        kwargs.update(dict(pyramid_level=1, extra_convs=True, softmax_temperature=10.0))

    tapir = tapir_model.ParameterizedTAPIR(params, state, tapir_kwargs=kwargs)

    print("Preprocessing video...")
    # Preprocess video for model
    frames = model_utils.preprocess_frames(video[None])  # Add batch dimension

    print("Compiling JAX functions...")
    # JIT compile the model with a small sample to initialize
    sample_points = query_points[: min(2, len(query_points))][None]  # Small sample with batch dim
    _ = tapir(
        video=frames[:, :2],  # Use first 2 frames
        query_points=sample_points,
        is_training=False,
        query_chunk_size=chunk_size,
    )
    print("JAX compilation complete.")

    # Run batch inference on entire video
    tracks, visibles = batch_inference(tapir, frames, query_points, chunk_size)

    print("Generating output video...")

    # Transform coordinates back to original video size if needed
    if original_video.shape[1:3] != (resize_size, resize_size):
        tracks = transforms.convert_grid_coordinates(
            tracks, (resize_size, resize_size), original_video.shape[1:3]
        )

    # Create visualization
    colormap = viz_utils.get_colors(len(detected_points))
    video_viz = viz_utils.paint_point_track(original_video, tracks, visibles, colormap)

    # Save output video
    media.write_video(output_path, video_viz, fps=fps)

    print(f"Video processing complete! Output saved to: {output_path}")
    print(f"Tracked {len(detected_points)} points across {len(video)} frames")


if __name__ == "__main__":
    main()
