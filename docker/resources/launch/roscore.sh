#!/bin/bash

source /opt/"${ORGANIZATION}"/scripts/lib/set_master.sh "${ROS_MASTER_INTERFACE}"
source "${BASE_ROS_WS_ROOT}"/install/setup.bash

roscore
