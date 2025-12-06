#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo apt-get update
sudo apt-get install -y libboost-all-dev libeigen3-dev liblz4-dev

source /opt/${ORGANIZATION}/venv/bin/activate
python -m pip install catkin catkin-pkg --extra-index-url https://woz4tetra.github.io/rospy-simple/
python -m pip install -r ${BASE_DIR}/ros-requirements.txt

python -c "import rospy"

echo "Installed ROS python packages for perception"
