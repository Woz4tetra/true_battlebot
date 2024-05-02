#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo apt-get update
sudo apt-get install -y libboost-all-dev libeigen3-dev
sudo -H python -m pip install --no-cache-dir pybind11==2.12.0
sudo -H python -m pip install --no-cache-dir 'git+https://github.com/Woz4tetra/genpy.git/@7ba2cfabaf6a9954a5c7db6118579d35f090207e'
sudo -H python -m pip install --no-cache-dir --ignore-installed -r ${BASE_DIR}/ros-requirements.txt

echo "Installed ROS python packages for perception"
