#!/bin/bash

set -e

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
source "${BASE_ROS_WS_ROOT}"/install/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true
catkin_make -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python

echo "Installed project dependencies"
