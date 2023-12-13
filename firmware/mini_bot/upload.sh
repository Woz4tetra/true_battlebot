#!/usr/bin/env bash

echo "Running mini_bot firmware upload script"

platformio run --target upload --upload-port=/dev/ttyACM0
