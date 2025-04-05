#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo apt-get update
sudo rosdep init
rosdep_update () {
    rosdep update
}
export -f rosdep_update
retry 10 rosdep_update

mkdir -p ${BASE_ROS_WS_ROOT}/src
cd ${BASE_ROS_WS_ROOT}

vcs import --retry 100 --input ${BASE_DIR}/${ROS_DISTRO}.rosinstall ./src

cd ${BASE_ROS_WS_ROOT}
git clone https://github.com/ros-infrastructure/catkin_pkg.git -b 0.5.2
cd catkin_pkg
python setup.py install --user

cd ${BASE_ROS_WS_ROOT}
git clone https://github.com/ros-infrastructure/rospkg.git -b 1.5.0
cd rospkg
python setup.py install --user

rosdep_install_dependencies() {
    cd ${BASE_ROS_WS_ROOT}
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true
}
export -f rosdep_install_dependencies
retry 10 rosdep_install_dependencies

echo "Installed ROS sources"
