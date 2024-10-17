#!/bin/bash
BASE_DIR=$(realpath "$(dirname "${0}")")
python "${BASE_DIR}"/test_video.py /data/training/raw_data/nhrl_video/Cage-7-Overhead-High-2024-09-14_18-15-43.049.mp4 $@ -i 3500 -n
python "${BASE_DIR}"/test_video.py /data/training/raw_data/nhrl_video/MobileCam-2-2024-09-14_13-06-39.141.mp4 $@ -i 5500 -n
python "${BASE_DIR}"/test_video.py /data/training/raw_data/nhrl_video/MobileCam-4-2024-09-14_13-12-25.478.mp4 $@ -i 0 -n
python "${BASE_DIR}"/test_video.py /data/training/raw_data/nhrl_video/Cage-6-Overhead-High-2024-09-14_13-40-53.873.mp4 $@ -i 5600 -n
