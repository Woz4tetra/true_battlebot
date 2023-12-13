#!/usr/bin/env bash

echo "Running mini_bot firmware monitor"

platformio device monitor --baud 115200 --port /dev/ttyACM0
