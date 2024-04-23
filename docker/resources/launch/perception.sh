#!/bin/bash
echo "Starting ${PROJECT_NAME} perception"

source /media/storage/robot
source /opt/${ORGANIZATION}/scripts/lib/set_master.sh ${ROS_MASTER_INTERFACE}

PROJECT_DIR=/opt/${ORGANIZATION}/${PROJECT_NAME}
python ${PROJECT_DIR}/perception/detection/main.py ${PROJECT_DIR}/perception/configs
