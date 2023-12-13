#!/usr/bin/env bash

echo "Running mini_bot firmware compile script"

export PLATFORMIO_WORKSPACE_DIR=${ROS_WS_ROOT}/.pio/mini_bot
platformio run
