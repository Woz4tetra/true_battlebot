#!/bin/bash
source /media/storage/robot

echo "Starting depthai_ros_driver"
echo "Robot name: ${ROBOT}"

if [ ${ROBOT} = "mini_bot" ]; then
    roslaunch \
        depthai_ros_driver \
        camera.launch \
        camera_model:=OAK-1-W \
        params_file:=/params/oak_1_w.yaml \
        parent_frame:=camera_1
else
    echo "DepthAI ROS driver is not supported for ${ROBOT}"
fi
