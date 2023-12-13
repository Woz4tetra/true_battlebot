#!/usr/bin/env bash

echo "Running mini_bot firmware upload script"

export PLATFORMIO_WORKSPACE_DIR=${ROS_WS_ROOT}/.pio/mini_bot
platformio run --target upload --upload-port=/dev/ttyACM0
