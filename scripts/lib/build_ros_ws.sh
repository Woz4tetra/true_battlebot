#!/bin/bash

set -e

BUILD_ARGS="${1}"
ROBOT_NAME="${2}"
MAP_NAME="${3}"

/opt/"${ORGANIZATION}"/scripts/set_robot "${ROBOT_NAME}" "${MAP_NAME}"
/opt/"${ORGANIZATION}"/scripts/lib/make_data_structure.sh

source "${BASE_ROS_WS_ROOT}"/install/setup.bash
source "${DEP_ROS_WS_ROOT}"/devel/setup.bash

cd "${ROS_WS_ROOT}"
catkin_make "${BUILD_ARGS}"
