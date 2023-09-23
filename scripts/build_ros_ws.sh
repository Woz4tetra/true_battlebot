#!/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash

cd ${ROS_WS_ROOT}
catkin_make $@
