#!/bin/bash

set -e

touch "${ROS_WS_ROOT}"/robot
source "${ROS_WS_ROOT}"/robot

if [ -z "${ROBOT}" ] || [ -n "${1}" ]
    then export ROBOT="${1}"
fi

if [ -z "${MAP_NAME}" ] || [ -n "${2}" ]
    then export MAP_NAME="${2}"
fi

if [ -z "${ROBOT}" ]
    then echo "Error: Provide robot name"
    exit 1
fi

if [ -z "${MAP_NAME}" ]
    then echo "Error: Provide map name"
    exit 1
fi

export FOUND_LAUNCHES=$(ls -1 /opt/"${ORGANIZATION}"/"${PROJECT_NAME}"/src/bw_bringup/launch)
MATCH_FOUND=$(python <<EOF
import os
found_launches = os.environ["FOUND_LAUNCHES"]
robot = os.environ["ROBOT"]
expected_path = robot + ".launch"
for launch in found_launches.splitlines():
    if expected_path == launch:
        print("yes")
        break
EOF
)
if [ -z "${MATCH_FOUND}" ]; then
    echo "Robot ${ROBOT} not found"
    exit 1
fi

cat <<EOT > "${ROS_WS_ROOT}"/robot
export ROBOT=${ROBOT}
export MAP_NAME=${MAP_NAME}
EOT

echo "Robot name: ${ROBOT}"
echo "Map name: ${MAP_NAME}"