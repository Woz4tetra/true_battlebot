#!/usr/bin/env python
"""
Demo script showing how to use the TAPIR auto-annotation functionality.
This script demonstrates the same capabilities as the TAPIR demo notebook
but as a standalone Python script with automatic point detection.
"""

import argparse
import os
import sys


def main():
    """Run the TAPIR auto-annotation demo."""
    parser = argparse.ArgumentParser(
        description="TAPIR Auto-Annotation Demo - recreates notebook functionality"
    )
    parser.add_argument(
        "-v",
        "--video",
        type=str,
        help="Path to input video file (if not provided, will download demo video)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Path to output video file",
    )
    parser.add_argument(
        "--download-demo",
        action="store_true",
        help="Download the demo horse jump video used in the notebook",
    )
    
    args = parser.parse_args()
    
    # Ensure we're in the right directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # Download demo video if requested or if no video provided
    if args.download_demo or args.video is None:
        print("Downloading demo video (horse jump)...")
        os.system("mkdir -p demo_videos")
        os.system("wget -P demo_videos http://storage.googleapis.com/dm-tapnet/horsejump-high.mp4")
        demo_video = "demo_videos/horsejump-high.mp4"
        
        if args.video is None:
            args.video = demo_video
    
    # Set default output path
    if args.output is None:
        video_name = args.video.rsplit(".", 1)[0]
        args.output = f"{video_name}_demo_auto_tracked.mp4"
    
    # Check if video file exists
    if not os.path.exists(args.video):
        print(f"Error: Video file not found: {args.video}")
        sys.exit(1)
    
    print("=" * 60)
    print("TAPIR Auto-Annotation Demo")
    print("=" * 60)
    print(f"Input video: {args.video}")
    print(f"Output video: {args.output}")
    print()
    print("This demo replicates the TAPIR notebook functionality:")
    print("1. Automatically detects interesting points to track")
    print("2. Uses motion detection, corner detection, and edge detection")
    print("3. Processes the entire video in batch mode (not frame-by-frame)")
    print("4. Uses efficient chunked inference like the notebook")
    print("5. Generates a video with tracked points colored and numbered")
    print()
    
    # Build command for the auto-annotation script
    cmd = [
        "python", "tapnet_auto_annotate_video.py",
        args.video,
        "-o", args.output,
        "-n", "30",  # Track 30 points like the notebook examples
        "-sz", "256",  # 256x256 processing size like notebook
        "--model-type", "bootstapir",  # Use bootstapir like notebook
    ]
    
    print("Running command:")
    print(" ".join(cmd))
    print()
    
    # Execute the auto-annotation
    result = os.system(" ".join(cmd))
    
    if result == 0:
        print()
        print("=" * 60) 
        print("Demo completed successfully!")
        print(f"Output video saved to: {args.output}")
        print()
        print("The output video shows:")
        print("- Automatically detected points as colored circles")
        print("- Point trajectories tracked across all frames")
        print("- Points are numbered for identification")
        print("- Occlusions are handled (points disappear when occluded)")
        print()
        print("This demonstrates the same core functionality as the TAPIR notebook")
        print("but with automatic point detection instead of manual selection.")
    else:
        print("Error occurred during processing. Check the output above for details.")
        sys.exit(1)


if __name__ == "__main__":
    main()