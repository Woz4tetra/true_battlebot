#!/bin/bash

set -e
sudo apt-get update

sudo apt-get install -y \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-clear-costmap-recovery \
    ros-${ROS_DISTRO}-move-slow-and-clear \
    ros-${ROS_DISTRO}-rotate-recovery

echo "Installed ROS project packages"
