#!/bin/bash
rosbag record -o /media/storage/bags/pose_label \
    /camera_0/rgb/image_rect_color \
    /camera_0/rgb/camera_info \
    /optical_camera_to_map_pose \
    /filter/field
