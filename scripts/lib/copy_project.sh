#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

echo "Copying project to ${ROS_WS_SRC}/${PROJECT_NAME} from /opt/${ORGANIZATION}/${PROJECT_NAME}"
sudo chown 1000:1000 "${ROS_WS_ROOT}"
mkdir -p "${ROS_WS_SRC}"
rsync -av --delete --exclude-from=/opt/"${ORGANIZATION}"/"${PROJECT_NAME}"/src/copy_exclude.txt /opt/"${ORGANIZATION}"/"${PROJECT_NAME}"/src/ "${ROS_WS_SRC}"/"${PROJECT_NAME}"
rsync -rtuv /opt/"${ORGANIZATION}"/"${PROJECT_NAME}"/src/bw_data/data/ "${ROS_WS_SRC}"/"${PROJECT_NAME}"/bw_data/data
rsync -rtuv "${ROS_WS_SRC}"/"${PROJECT_NAME}"/bw_data/data/ /opt/"${ORGANIZATION}"/"${PROJECT_NAME}"/src/bw_data/data

${BASE_DIR}/make_arduino_ide_structure.sh
