#!/bin/bash

set -e
sudo apt-get update

sudo apt-get install -y \
    ros-${ROS_DISTRO}-teleop-twist-joy

echo "Installed ROS project packages"
