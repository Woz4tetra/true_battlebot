#!/bin/bash
set -e

source "${BASE_ROS_WS_ROOT}"/devel/setup.bash
source "${DEP_ROS_WS_ROOT}"/devel/setup.bash
source "${ROS_WS_ROOT}"/devel/setup.bash
source ${ROS_WS_ROOT}/robot

# https://stackoverflow.com/questions/58150251/numpy-matrix-inverse-appears-to-use-multiple-threads
export OPENBLAS_NUM_THREADS=1
export ROSCONSOLE_FORMAT='[${node}] [${severity}] [${time}]: ${message}'

if [ -n "${REMOTE_MACHINE}" ]; then
    source /opt/"${ORGANIZATION}"/scripts/set_client "${REMOTE_MACHINE}"
fi

exec "$@"
