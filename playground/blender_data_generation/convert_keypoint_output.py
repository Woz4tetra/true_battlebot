import argparse
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np


def find_all_blobs(image: np.ndarray) -> List[Dict]:
    """Find all colored blobs in the image using connected components."""
    # Extract only non-transparent pixels (alpha == 0xFFFF for 16-bit)
    alpha_mask = (image[:, :, 3] == 0xFFFF).astype(np.uint8) * 255

    # Find all connected components in the alpha mask
    num_labels, labels = cv2.connectedComponents(alpha_mask, connectivity=8)

    blobs = []
    # Skip label 0 (background)
    for label_id in range(1, num_labels):
        # Get mask for this blob
        blob_mask = labels == label_id

        # Get all pixel coordinates for this blob
        coords = np.column_stack(np.where(blob_mask))
        # Convert from (y, x) to (x, y) format
        coords_list = [(int(x), int(y)) for y, x in coords]

        # Calculate centroid
        centroid_y, centroid_x = coords.mean(axis=0)

        # Get average color of this blob (in 16-bit BGR space)
        blob_pixels = image[blob_mask]
        avg_color = blob_pixels[:, :3].mean(axis=0)

        blobs.append(
            {
                "coords": coords_list,
                "centroid": (float(centroid_x), float(centroid_y)),
                "avg_color": avg_color,
                "num_pixels": len(coords_list),
            }
        )

    return blobs


def match_blob_to_global_id(blob_color: np.ndarray, global_colors: List[np.ndarray], tolerance: int = 2000) -> int:
    """Match a blob's average color to a global blob ID."""
    for idx, global_color in enumerate(global_colors):
        # Compare colors with tolerance (16-bit space)
        color_diff = np.abs(blob_color.astype(np.int32) - global_color.astype(np.int32))
        if np.all(color_diff < tolerance):
            return idx

    # No match found, add as new color
    global_colors.append(blob_color.copy())
    return len(global_colors) - 1


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_dir", type=str)
    parser.add_argument("--background-image-dir", required=False, type=str)
    parser.add_argument("--show-debug", action="store_true", help="Show individual blob masks")
    parser.add_argument("--color-tolerance", type=int, default=2000, help="Color matching tolerance (16-bit)")
    args = parser.parse_args()

    image_dir = Path(args.image_dir)
    background_image_dir = Path(args.background_image_dir) if args.background_image_dir else None
    show_debug = args.show_debug
    color_tolerance = args.color_tolerance

    # Global list to track blob colors across all images
    global_colors: List[np.ndarray] = []

    # Store results for all images
    all_results = {}

    # Sort image paths for consistent ordering
    image_paths = sorted([p for p in image_dir.iterdir() if p.suffix == ".png"])
    background_image_paths = (
        sorted([p for p in background_image_dir.iterdir() if p.suffix == ".png"]) if background_image_dir else []
    )
    background_image_stems = [p.stem for p in background_image_paths]

    for image_path in image_paths:
        print(f"\nProcessing: {image_path.name}")

        if show_debug and image_path.stem in background_image_stems:
            background_image_path = background_image_paths[background_image_stems.index(image_path.stem)]
            background_image = cv2.imread(str(background_image_path))
        else:
            background_image = None

        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            print("  Failed to load image")
            continue

        # Find all blobs in the image
        blobs = find_all_blobs(image)
        print(f"  Found {len(blobs)} blobs")

        # Map blobs to global IDs based on color similarity
        image_keypoints = {}
        for blob in blobs:
            global_id = match_blob_to_global_id(blob["avg_color"], global_colors, color_tolerance)

            image_keypoints[global_id] = {
                "color": blob["avg_color"].tolist(),  # BGR values (16-bit)
                "coords": blob["coords"],
                "centroid": blob["centroid"],
                "num_pixels": blob["num_pixels"],
            }

            cx, cy = blob["centroid"]
            print(f"  Blob ID {global_id}: {blob['num_pixels']} pixels, centroid at ({cx:.1f}, {cy:.1f})")

        all_results[image_path.name] = image_keypoints

        # Debug visualization showing individual blobs
        if background_image is not None and len(image_keypoints) > 0:
            vis = background_image
            for global_id, data in image_keypoints.items():
                centroid = data["centroid"]
                cx, cy = int(centroid[0]), int(centroid[1])

                # Draw circle at centroid
                cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
                # Draw ID label with background
                text = str(global_id)
                (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(vis, (cx + 10, cy - text_h - 5), (cx + 10 + text_w, cy + 5), (0, 0, 0), -1)
                cv2.putText(
                    vis,
                    text,
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

            cv2.imshow("Keypoints", background_image)
            print("  Press any key to continue, 'q' to quit...")
            key = chr(cv2.waitKey(0) & 0xFF)
            if key == "q":
                quit()

    if show_debug or show_debug:
        cv2.destroyAllWindows()

    # Print summary
    print(f"\n{'=' * 60}")
    print(f"SUMMARY: Found {len(global_colors)} unique blobs across all images")
    print(f"{'=' * 60}")

    for img_name, keypoints in all_results.items():
        print(f"\n{img_name}:")
        for blob_id in sorted(keypoints.keys()):
            data = keypoints[blob_id]
            cx, cy = data["centroid"]
            print(f"  Blob {blob_id}: centroid=({cx:.1f}, {cy:.1f}), pixels={data['num_pixels']}")


if __name__ == "__main__":
    main()
