#!/usr/bin/env python
"""
Demo script showing the complete workflow:
1. Label points interactively with the UI tool
2. Use those points for TAPIR tracking
"""

import os
import subprocess
import sys


def main():
    if len(sys.argv) != 2:
        print("Usage: python demo_workflow.py <video_path>")
        print()
        print("This script demonstrates the complete labeling -> tracking workflow:")
        print("1. Opens interactive labeling tool")
        print("2. Uses labeled points for TAPIR tracking")
        return

    video_path = sys.argv[1]

    if not os.path.exists(video_path):
        print(f"Error: Video file not found: {video_path}")
        return

    print("=== TAPIR Point Labeling & Tracking Workflow ===")
    print(f"Video: {video_path}")
    print()

    # Step 1: Interactive labeling
    print("Step 1: Interactive Point Labeling")
    print("- Use the labeling tool to select points to track")
    print("- Left click to add points, right click to remove")
    print("- Press 'S' to save, 'Q' to quit")
    print()

    choice = input("Start interactive labeling? (y/N): ").lower().strip()
    if choice != "y":
        print("Skipping labeling step.")
        return

    # Run the labeling tool
    print("Starting labeling tool...")
    result = subprocess.run(
        ["python", "tapnet_point_labeler.py", video_path], cwd=os.path.dirname(os.path.abspath(__file__))
    )

    if result.returncode != 0:
        print("Labeling tool failed or was cancelled.")
        return

    # Step 2: Check for generated files
    video_name = os.path.splitext(os.path.basename(video_path))[0]

    # Check for trimmed video and CSV
    trimmed_video_path = f"{video_name}_trimmed.mp4"
    csv_path = f"{video_name}_trimmed_points.csv"

    # Fallback to original video and regular CSV if trimmed not found
    if not os.path.exists(trimmed_video_path):
        trimmed_video_path = video_path
        csv_path = f"{video_name}_points.csv"

    if not os.path.exists(csv_path):
        print(f"No CSV file found: {csv_path}")
        print("Make sure you saved points in the labeling tool (press 'S').")
        return

    print(f"\nFound labeled points: {csv_path}")
    print(f"Video to process: {trimmed_video_path}")

    # Step 3: Run TAPIR tracking
    print("\nStep 2: TAPIR Tracking")
    print("- Using labeled points for tracking")
    print("- Processing entire video with batch inference")
    print()

    choice = input("Start TAPIR tracking? (y/N): ").lower().strip()
    if choice != "y":
        print("Skipping tracking step.")
        return

    output_video = f"{video_name}_labeled_tracked.mp4"

    print("Starting TAPIR tracking...")
    result = subprocess.run(
        [
            "python",
            "tapnet_auto_annotate_video.py",
            trimmed_video_path,  # Use trimmed video if available
            "--points-csv",
            csv_path,
            "-o",
            output_video,
            "-sz",
            "256",
        ],
        cwd=os.path.dirname(os.path.abspath(__file__)),
    )

    if result.returncode == 0:
        print(f"\n🎉 Success! Tracked video saved to: {output_video}")
        print("\nWorkflow complete!")
        print(f"- Labeled points: {csv_path}")
        print(f"- Tracked video: {output_video}")
    else:
        print("TAPIR tracking failed. Check the error messages above.")


if __name__ == "__main__":
    main()
