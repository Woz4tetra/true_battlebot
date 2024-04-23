#!/bin/bash

BASE_DIR=$(realpath "$(dirname $0)")

PROJECT_DIR=$(realpath "${BASE_DIR}/../../")

RM_PATTERN="$1"

if [ -z "${RM_PATTERN}" ]; then
    echo "Removal pattern is empty! Exiting."
    exit 1
fi

rm -r ${ROS_WS_SRC}/${PROJECT_NAME}/bw_data/data/"${RM_PATTERN}"
rm -r ${PROJECT_DIR}/${PROJECT_NAME}/ros_ws/bw_data/data/"${RM_PATTERN}"
