#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo rosdep init
rosdep update

mkdir -p ${BASE_ROS_WS_ROOT}/src
cd ${BASE_ROS_WS_ROOT}

vcs import --input ${BASE_DIR}/${ROS_DISTRO}.rosinstall ./src

cd ${BASE_ROS_WS_ROOT}
git clone https://github.com/ros-infrastructure/catkin_pkg.git -b 0.5.2
cd catkin_pkg
python setup.py install --user

cd ${BASE_ROS_WS_ROOT}
git clone https://github.com/ros-infrastructure/rospkg.git -b 1.5.0
cd rospkg
python setup.py install --user

cd ${BASE_ROS_WS_ROOT}
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true

./src/catkin/bin/catkin_make install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python

echo "Installed ROS"
