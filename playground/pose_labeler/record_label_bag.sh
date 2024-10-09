#!/bin/bash
rosbag record -o /data/bags/pose_label \
    /camera_0/rgb/image_raw \
    /camera_0/rgb/camera_info \
    /optical_camera_to_map_pose \
    /filter/field
