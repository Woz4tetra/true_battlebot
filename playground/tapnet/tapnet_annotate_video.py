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

"""Video annotation with TAPIR point tracking."""

import argparse
import time
from dataclasses import dataclass

import cv2
import jax
import jax.numpy as jnp
import numpy as np
import tqdm
from tapnet.models import tapir_model
from tapnet.utils import model_utils


@dataclass
class AppData:
    frame: np.ndarray | None
    last_click_time: float
    pos: tuple[int, int]
    query_frame: bool
    selected_points: list[tuple[int, int]]
    selection_mode: bool


def load_checkpoint(checkpoint_path):
    ckpt_state = np.load(checkpoint_path, allow_pickle=True).item()
    return ckpt_state["params"], ckpt_state["state"]


def online_model_init(tapir, frames, points):
    feature_grids = tapir.get_feature_grids(frames, is_training=False)
    features = tapir.get_query_features(
        frames,
        is_training=False,
        query_points=points,
        feature_grids=feature_grids,
    )
    return features


def online_model_predict(tapir, frames, features, causal_context):
    """Compute point tracks and occlusions given frames and query points."""
    feature_grids = tapir.get_feature_grids(frames, is_training=False)
    trajectories = tapir.estimate_trajectories(
        frames.shape[-3:-1],
        is_training=False,
        feature_grids=feature_grids,
        query_features=features,
        query_points_in_video=None,
        query_chunk_size=64,
        causal_context=causal_context,
        get_causal_context=True,
    )
    causal_context = trajectories["causal_context"]
    del trajectories["causal_context"]
    return {k: v[-1] for k, v in trajectories.items()}, causal_context


def get_frame_original(video_capture, output_width):
    """Get frame in original resolution without cropping."""
    ret, frame = video_capture.read()
    if not ret:
        return ret, frame
    if output_width is not None and frame.shape[1] != output_width:
        aspect_ratio = frame.shape[0] / frame.shape[1]
        new_height = int(output_width * aspect_ratio)
        frame = cv2.resize(frame, (output_width, new_height))
    return ret, frame


def squash_frame(frame, new_size):
    """Get frame squashed to square for model processing."""
    return cv2.resize(frame, new_size)


def transform_coordinates_to_model(points, original_shape, model_shape):
    """Transform coordinates from original video size to model input size."""
    orig_h, orig_w = original_shape[:2]
    model_h, model_w = model_shape[:2]

    transformed_points = []
    for y, x in points:
        # Convert from original coordinates to squashed model coordinates
        model_x = (x / orig_w) * model_w
        model_y = (y / orig_h) * model_h
        transformed_points.append((model_y, model_x))

    return transformed_points


def transform_coordinates_from_model(points, model_shape, original_shape):
    """Transform coordinates from model output back to original video size."""
    orig_h, orig_w = original_shape[:2]
    model_h, model_w = model_shape[:2]

    transformed_points = []
    for model_x, model_y in points:
        x = (model_x / model_w) * orig_w
        y = (model_y / model_h) * orig_h
        transformed_points.append((y, x))

    return transformed_points


def mouse_click_selection(event, x, y, flags, param):
    """Mouse callback for point selection mode."""
    app_data: AppData = param[0]
    last_click_time = app_data.last_click_time

    # event fires multiple times per click sometimes??
    if (time.time() - last_click_time) < 0.5:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        # Store point as (row, col) in original video coordinates
        # No coordinate flip needed since we're not flipping the selection display
        point = (y, x)
        app_data.selected_points.append(point)
        app_data.last_click_time = time.time()
        print(f"Selected point {len(app_data.selected_points)}: ({x}, {y})")


def mouse_click(event, x, y, flags, param):
    app_data: AppData = param[0]
    frame = app_data.frame
    last_click_time = app_data.last_click_time

    # event fires multiple times per click sometimes??
    if (time.time() - last_click_time) < 0.5:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        app_data.pos = (y, frame.shape[1] - x)
        app_data.query_frame = True
        last_click_time = time.time()


def select_points_on_frame(original_frame):
    """Display the frame at original resolution and allow user to select points to track."""
    print("Point Selection Mode:")
    print("- Left click to select points to track")
    print("- Press ENTER when done selecting points")
    print("- Press ESC to cancel")

    app_data = AppData(
        frame=original_frame,
        last_click_time=0.0,
        pos=(),
        query_frame=False,
        selected_points=[],
        selection_mode=True,
    )

    cv2.namedWindow("Select Points to Track")
    cv2.setMouseCallback("Select Points to Track", mouse_click_selection, param=[app_data])

    while True:
        # Draw selected points on the display frame
        display_frame = original_frame.copy()
        for i, point in enumerate(app_data.selected_points):
            # Use original coordinates directly (no flip)
            y, x = point
            cv2.circle(display_frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(display_frame, str(i + 1), (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display without flipping
        cv2.imshow("Select Points to Track", display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 13:  # Enter key
            break
        elif key == 27:  # ESC key
            app_data.selected_points = []
            break

    cv2.destroyWindow("Select Points to Track")
    return app_data.selected_points


def main() -> None:
    parser = argparse.ArgumentParser(description="Annotate video with TAPIR point tracking")
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
        help="Path to the output video file. If not specified, will use input path with '_tracked' suffix.",
    )
    parser.add_argument(
        "-ch",
        "--checkpoint_path",
        type=str,
        default="/opt/deepmind/tapnet/checkpoints/causal_tapir_checkpoint.npy",
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
        "-x",
        "--size",
        type=int,
        default=480,
        help="Size to which the input video frames will be resized (square).",
    )
    parser.add_argument(
        "-ow",
        "--output-width",
        type=int,
        default=None,
        help="Width of the output video frames. If not specified, uses original video width.",
    )
    args = parser.parse_args()
    square_size = args.size
    output_width = args.output_width

    video_path = args.video_path
    checkpoint_path = args.checkpoint_path
    start_time = args.start_time
    end_time = args.end_time

    # Generate output path if not specified
    if args.output_path is None:
        video_name = video_path.rsplit(".", 1)[0]
        video_ext = video_path.rsplit(".", 1)[1] if "." in video_path else "mp4"
        output_path = f"{video_name}_tracked.{video_ext}"
    else:
        output_path = args.output_path

    print(f"Input video: {video_path}")
    print(f"Output video: {output_path}")

    # Open video file
    vc = cv2.VideoCapture(video_path)
    if not vc.isOpened():
        raise ValueError(f"Unable to open video file: {video_path}")

    # Get video properties
    fps = vc.get(cv2.CAP_PROP_FPS)
    frame_width = int(vc.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(vc.get(cv2.CAP_PROP_FRAME_COUNT))

    min_dim = min(frame_width, frame_height, square_size, output_width if output_width is not None else frame_width)
    # round down to nearest multiple of 8
    new_min_dim = (min_dim // 8) * 8
    if new_min_dim != min_dim:
        print(f"Adjusting model input size from {min_dim} to {new_min_dim} to be multiple of 8")
        min_dim = new_min_dim
    model_input_size = (min_dim, min_dim)

    print(f"Video properties: {frame_width}x{frame_height}, {fps} fps, {total_frames} frames")
    print(f"Model input size: {model_input_size[0]}x{model_input_size[1]}")
    initial_frame = int(start_time * fps)
    if end_time is not None:
        final_frame = int(end_time * fps)
        total_frames = min(total_frames, final_frame)
    else:
        final_frame = total_frames
    print(f"Processing frames from {initial_frame} to {total_frames}")

    # Get the first frame in original resolution for point selection
    vc.set(cv2.CAP_PROP_POS_FRAMES, initial_frame)  # Reset to initial frame
    rval, first_frame_original = get_frame_original(vc, output_width)
    print(f"First frame shape (original): {first_frame_original.shape}")
    if not rval:
        raise ValueError("Unable to read first frame from video")

    # Let user select points on original frame
    selected_points_original = select_points_on_frame(first_frame_original)
    if not selected_points_original:
        print("No points selected. Exiting.")
        vc.release()
        return

    num_points = len(selected_points_original)
    print(f"Selected {num_points} points for tracking")

    # Get the cropped frame for model processing
    first_frame_squashed = squash_frame(first_frame_original, model_input_size)

    # Transform selected points to model coordinates
    selected_points_model = transform_coordinates_to_model(
        selected_points_original, first_frame_original.shape, first_frame_squashed.shape
    )

    if len(selected_points_model) != num_points:
        print(f"Warning: Some points were outside the cropped region. Using {len(selected_points_model)} points.")
        num_points = len(selected_points_model)

    print("Loading checkpoint...")
    # Load checkpoint and initialize
    params, state = load_checkpoint(checkpoint_path)

    tapir = tapir_model.ParameterizedTAPIR(
        params=params,
        state=state,
        tapir_kwargs=dict(use_causal_conv=True, bilinear_interp_with_depthwise_conv=False),
    )

    print("Creating model...")
    online_init_apply = jax.jit(lambda frames, points: online_model_init(tapir, frames, points))
    online_predict_apply = jax.jit(
        lambda frames, features, causal_context: online_model_predict(tapir, frames, features, causal_context)
    )

    print("Compiling jax functions (this may take a while...)")
    # Call one time to compile with selected points
    query_points = jnp.array([(0,) + point for point in selected_points_model], dtype=jnp.float32)

    query_features = online_init_apply(
        frames=model_utils.preprocess_frames(first_frame_squashed[None, None]),
        points=query_points[None, :],
    )
    jax.block_until_ready(query_features)

    causal_state = tapir.construct_initial_causal_state(num_points, len(query_features.resolutions) - 1)

    prediction, causal_state = online_predict_apply(
        frames=model_utils.preprocess_frames(first_frame_squashed[None, None]),
        features=query_features,
        causal_context=causal_state,
    )
    jax.block_until_ready(prediction["tracks"])

    # Setup video writer (use original frame dimensions)
    output_height, output_width = first_frame_original.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (output_width, output_height))

    print("Processing video...")
    vc.set(cv2.CAP_PROP_POS_FRAMES, initial_frame)  # Reset to initial frame

    for _ in tqdm.tqdm(range(initial_frame, total_frames), desc="Processing frames"):
        # Get both original and cropped frames
        rval_orig, frame_original = get_frame_original(vc, output_width)
        if not rval_orig:
            break

        frame_squared = squash_frame(frame_original, model_input_size)

        # Predict tracks for current frame (use cropped frame for model)
        prediction, causal_state = online_predict_apply(
            frames=model_utils.preprocess_frames(frame_squared[None, None]),
            features=query_features,
            causal_context=causal_state,
        )

        track = prediction["tracks"][0, :, 0]
        occlusion = prediction["occlusion"][0, :, 0]
        expected_dist = prediction["expected_dist"][0, :, 0]
        visibles = model_utils.postprocess_occlusions(occlusion, expected_dist)

        # Transform model coordinates back to original frame coordinates
        model_points = [(float(track[i, 0]), float(track[i, 1])) for i in range(num_points)]
        original_points = transform_coordinates_from_model(
            model_points, first_frame_squashed.shape, first_frame_original.shape
        )

        # Draw tracked points on original frame
        annotated_frame = frame_original.copy()
        for i in range(num_points):
            if visibles[i]:
                y, x = original_points[i]
                x, y = int(round(x)), int(round(y))
                cv2.circle(annotated_frame, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(annotated_frame, str(i + 1), (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Write frame to output video (no horizontal flip)
        out.write(annotated_frame)

    # Cleanup
    vc.release()
    out.release()

    print(f"Video processing complete! Output saved to: {output_path}")


if __name__ == "__main__":
    main()
