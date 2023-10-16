#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true
