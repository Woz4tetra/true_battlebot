#!/bin/bash

set -e

sudo chown 1000:1000 ${ROS_WS_ROOT}
mkdir -p ${ROS_WS_SRC}
rsync -av --delete --exclude-from=/opt/${ORGANIZATION}/scripts/copy_exclude.txt /opt/${ORGANIZATION}/${PROJECT_NAME}/src/ ${ROS_WS_SRC}/${PROJECT_NAME}
rsync -rtuv /opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_data/data/ ${ROS_WS_SRC}/${PROJECT_NAME}/bw_data/data
rsync -rtuv ${ROS_WS_SRC}/${PROJECT_NAME}/bw_data/data/ /opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_data/data
