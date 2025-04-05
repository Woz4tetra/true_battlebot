#!/bin/bash

set -e

cd ${BASE_ROS_WS_ROOT}

sudo apt-get update
./src/catkin/bin/catkin_make install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/usr/bin/python

echo "Installed ROS"
