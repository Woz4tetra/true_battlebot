#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash
source /opt/${ORGANIZATION}/robot

# https://stackoverflow.com/questions/58150251/numpy-matrix-inverse-appears-to-use-multiple-threads
export OPENBLAS_NUM_THREADS=1
export ROSCONSOLE_FORMAT='[${node}] [${severity}] [${time}]: ${message}'

if [ ! -z ${REMOTE_MACHINE} ]; then
    source /opt/${ORGANIZATION}/scripts/set_client ${REMOTE_MACHINE}
fi

exec "$@"
