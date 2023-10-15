#!/bin/bash

set -e

export ROBOT="${1}"

if [ -z "${ROBOT}" ]
    then echo "Provide robot name"
    exit
fi

export MAP_NAME="${2}"

if [ -z "${MAP_NAME}" ]
    then echo "Provide map name"
    exit
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
