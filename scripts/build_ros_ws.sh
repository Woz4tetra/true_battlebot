#!/bin/bash

set -e

BUILD_ARGS="${1}"

ROBOT_NAME="${2}"

if [ -z "${ROBOT_NAME}" ]
    then echo "Provide robot name"
    exit
fi

MAP_NAME="${3}"

if [ -z "${MAP_NAME}" ]
    then echo "Provide map name"
    exit
fi

/opt/${ORGANIZATION}/scripts/set_robot "${ROBOT_NAME}" "${MAP_NAME}"

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash

cd ${ROS_WS_ROOT}
catkin_make "${BUILD_ARGS}"
