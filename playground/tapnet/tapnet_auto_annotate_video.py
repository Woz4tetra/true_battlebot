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
import csv
from typing import List, Optional, Tuple

import cv2
import numpy as np
import tqdm
from tapnet.models import tapir_model
from tapnet.utils import model_utils, transforms, viz_utils


def load_checkpoint(checkpoint_path):
    """Load TAPIR checkpoint."""
    ckpt_state = np.load(checkpoint_path, allow_pickle=True).item()
    return ckpt_state["params"], ckpt_state["state"]


def read_video_opencv(video_path: str, max_frames: Optional[int] = None) -> Tuple[np.ndarray, float]:
    """Read video using OpenCV instead of mediapy for better performance."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Could not open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = []

    print("Loading video frames...")
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if max_frames is not None:
        frame_count = min(frame_count, max_frames)
        print(f"Limiting to first {max_frames} frames for testing")

    with tqdm.tqdm(total=frame_count, desc="Reading frames") as pbar:
        for i in range(frame_count):
            ret, frame = cap.read()
            if not ret:
                break

            # Convert BGR to RGB for consistency with original code
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frames.append(frame_rgb)
            pbar.update(1)

    cap.release()
    return np.array(frames), fps


def resize_video_opencv(video: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
    """Resize video using OpenCV."""
    height, width = target_size
    resized_frames = []

    print(f"Resizing {len(video)} frames to {width}x{height}...")
    for frame in tqdm.tqdm(video, desc="Resizing frames"):
        resized = cv2.resize(frame, (width, height))
        resized_frames.append(resized)

    return np.array(resized_frames)


def write_video_opencv(output_path: str, video: np.ndarray, fps: float):
    """Write video using OpenCV."""
    height, width = video.shape[1:3]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print(f"Writing video with {len(video)} frames...")
    for frame in tqdm.tqdm(video, desc="Writing frames"):
        # Convert RGB back to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        out.write(frame_bgr)

    out.release()


def load_points_from_csv(csv_path: str) -> List[Tuple[int, int, int]]:
    """
    Load manually labeled points from CSV file.

    CSV format: point_id, frame, x, y
    Returns: List of (frame_idx, y, x) tuples
    """
    points = []

    with open(csv_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            frame_idx = int(row["frame"])
            x = int(row["x"])
            y = int(row["y"])
            points.append((frame_idx, y, x))  # Convert to (frame, y, x) format

    print(f"Loaded {len(points)} points from {csv_path}")
    return points


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
    gray_frames = [cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) for frame in frames[: min(10, len(frames))]]

    detected_points = []

    if use_motion and len(gray_frames) > 1:
        # Method 1: Motion-based detection
        print("  Using motion-based detection...")
        motion_points = detect_motion_points(gray_frames, num_points // 2, quality_level, min_distance)
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
        grid_points = sample_grid_points(gray_frames[0], num_points - len(detected_points), detected_points)
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
        # Use frame difference to detect motion areas
        frame_diff = cv2.absdiff(gray_frames[0], gray_frames[i])

        # Threshold the difference to create motion mask
        _, motion_mask = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        motion_mask = cv2.morphologyEx(motion_mask, cv2.MORPH_OPEN, kernel)
        motion_mask = cv2.morphologyEx(motion_mask, cv2.MORPH_CLOSE, kernel)

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
        indices = np.random.choice(len(edge_points[0]), min(num_points, len(edge_points[0])), replace=False)
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

    for chunk_start in tqdm.tqdm(range(0, num_points, chunk_size), desc="Processing point chunks"):
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
    parser = argparse.ArgumentParser(description="Auto-annotate video with TAPIR point tracking")
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
    parser.add_argument(
        "--points-csv",
        type=str,
        default=None,
        help="Path to CSV file with manually labeled points. If provided, skips automatic detection.",
    )

    args = parser.parse_args()

    video_path = args.video_path
    checkpoint_path = args.checkpoint_path
    resize_size = args.size
    num_points = args.num_points
    chunk_size = args.chunk_size
    model_type = args.model_type
    use_motion = not args.no_motion
    points_csv = args.points_csv

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
    # Limit to 30 frames for debugging
    video, fps = read_video_opencv(video_path, max_frames=30)
    print(f"Video shape: {video.shape}")
    original_video_size = video.shape[1:3]

    # Resize video for processing
    if video.shape[1] != resize_size or video.shape[2] != resize_size:
        print(f"Resizing video to {resize_size}x{resize_size}...")
        original_video = video.copy()
        video = resize_video_opencv(video, (resize_size, resize_size))
    else:
        original_video = video.copy()

    # Get points either from CSV or automatic detection
    if points_csv is not None:
        print(f"Loading points from CSV: {points_csv}")
        detected_points = load_points_from_csv(points_csv)

        # Debug: Show original points
        print(f"Original video shape: {original_video.shape}")
        print(f"Processing size: {resize_size}x{resize_size}")
        if detected_points:
            sample_point = detected_points[0]
            print(f"Sample original point: frame={sample_point[0]}, y={sample_point[1]}, x={sample_point[2]}")

        # Transform points if video was resized
        if original_video_size != (resize_size, resize_size):
            transformed_points = []
            orig_h, orig_w = original_video_size
            scale_x = resize_size / orig_w
            scale_y = resize_size / orig_h

            print(f"Coordinate scaling: x_scale={scale_x:.4f}, y_scale={scale_y:.4f}")

            for frame_idx, y, x in detected_points:
                # Scale coordinates from original to resized dimensions
                new_x = int(x * scale_x)
                new_y = int(y * scale_y)
                transformed_points.append((frame_idx, new_y, new_x))

            detected_points = transformed_points
            print(f"Transformed {len(detected_points)} points for resized video")

            # Debug: Show transformed points
            if detected_points:
                sample_point = detected_points[0]
                print(f"Sample transformed point: frame={sample_point[0]}, y={sample_point[1]}, x={sample_point[2]}")
        else:
            print("No coordinate transformation needed - video already at processing size")
    else:
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

    # Debug: Show query points being sent to TAPIR
    print("Query points sent to TAPIR (t, y, x format):")
    for i, (t, y, x) in enumerate(query_points[:3]):  # Show first 3 points
        print(f"  Point {i}: t={t}, y={y:.1f}, x={x:.1f}")

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

    # Debug: Show tracks before transformation
    if len(tracks) > 0:
        sample_track = tracks[0, 0]  # First point, first frame
        print(f"Sample track before coordinate transform: x={sample_track[0]:.2f}, y={sample_track[1]:.2f}")

        # Show tracks for first 3 points, first frame
        print("First few tracks (x, y format):")
        for i in range(min(3, len(tracks))):
            track = tracks[i, 0]
            print(f"  Track {i}: x={track[0]:.1f}, y={track[1]:.1f}")

    # Transform coordinates back to original video size if needed
    if original_video_size != (resize_size, resize_size):
        print(f"Converting coordinates back from {(resize_size, resize_size)} to {original_video_size}")
        tracks = transforms.convert_grid_coordinates(tracks, (resize_size, resize_size), original_video_size)

        # Debug: Show tracks after transformation
        if len(tracks) > 0:
            sample_track = tracks[0, 0]  # First point, first frame
            print(f"Sample track after coordinate transform: x={sample_track[0]:.2f}, y={sample_track[1]:.2f}")

            # Show tracks for first 3 points, first frame
            print("First few tracks after transformation (x, y format):")
            for i in range(min(3, len(tracks))):
                track = tracks[i, 0]
                print(f"  Track {i}: x={track[0]:.1f}, y={track[1]:.1f}")
    else:
        print("No coordinate transformation needed - video already at processing size")

    # Create visualization
    colormap = viz_utils.get_colors(len(detected_points))
    video_viz = viz_utils.paint_point_track(original_video, tracks, visibles, colormap)

    # Save output video
    write_video_opencv(output_path, video_viz, fps)

    print(f"Video processing complete! Output saved to: {output_path}")
    print(f"Tracked {len(detected_points)} points across {len(video)} frames")


if __name__ == "__main__":
    main()
