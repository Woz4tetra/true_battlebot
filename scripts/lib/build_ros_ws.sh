#!/bin/bash

set -e

BUILD_ARGS="${1}"
ROBOT_NAME="${2}"
MAP_NAME="${3}"

/opt/"${ORGANIZATION}"/scripts/lib/set_robot.sh "${ROBOT_NAME}" "${MAP_NAME}"

source /opt/ros/"${ROS_DISTRO}"/setup.bash
source "${DEP_ROS_WS_ROOT}"/devel/setup.bash

cd "${ROS_WS_ROOT}"
catkin_make "${BUILD_ARGS}"
