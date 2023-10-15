#!/bin/bash

set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update

sudo apt-get install -y --ignore-missing \
    ros-${ROS_DISTRO}-rospy \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-roscpp \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-roslaunch \
    ros-${ROS_DISTRO}-message-runtime \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-image-common \
    ros-${ROS_DISTRO}-marker-msgs \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-ros-numpy \
    ros-${ROS_DISTRO}-rviz \
    libsdl-image1.2-dev \
    libsdl-dev \
    python3-pip \
    python3-tk \
    python3-venv \
    v4l-utils \
    qtbase5-dev \
    qtdeclarative5-dev \
    jq

sudo apt-get upgrade -y

echo "Installed all basic apt packages"
