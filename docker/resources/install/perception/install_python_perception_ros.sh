#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo apt-get update
sudo apt-get install -y libboost-all-dev libeigen3-dev
sudo -H python -m pip install pybind11==2.12.0
sudo -H python -m pip install catkin catkin-pkg --extra-index-url https://rospypi.github.io/simple/
sudo -H python -m pip install -r ${BASE_DIR}/ros-requirements.txt

python -c "import rospy"

echo "Installed ROS python packages for perception"
