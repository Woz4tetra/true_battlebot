#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-program-options-dev \
    libconsole-bridge-dev \
    libpoco-dev \
    libtinyxml2-dev \
    liblz4-dev \
    libbz2-dev \
    uuid-dev \
    liblog4cxx-dev \
    libgpgme-dev \
    libgtest-dev \
    libbullet-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libnetpbm10-dev  \
    libtheora-dev  \
    libgeographic-dev  \
    libavutil-dev  \
    libswscale-dev  \
    liburdfdom-headers-dev  \
    liburdfdom-dev  \
    libsuitesparse-dev \
    python3-termcolor \
    libceres-dev \
    libgeos-dev \
    python3-vcstool

sudo python -m pip install -U rosdep
sudo rosdep init
rosdep update

BASE_DIR=$(realpath "$(dirname "${0}")")
ROSINSTALL_PATH=$1

mkdir -p ${DEP_ROS_WS_ROOT}/src
cd ${DEP_ROS_WS_ROOT}
vcs import --input ${ROSINSTALL_PATH} ./src

cd ${DEP_ROS_WS_ROOT}/src/geometry2/
git apply --ignore-whitespace ${BASE_DIR}/geometry2.patch
cd ${DEP_ROS_WS_ROOT}/src/image_pipeline/
git apply --ignore-whitespace ${BASE_DIR}/image-pipeline.patch

cd ${DEP_ROS_WS_ROOT}
source /opt/ros/"${ROS_DISTRO}"/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true
catkin_make install -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python
catkin_make -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python

echo "Installed project dependencies"
