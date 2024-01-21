#!/bin/bash

set -e

mkdir -p ${ROS_WS_ROOT}/arduino-ide/.arduinoIDE
mkdir -p ${ROS_WS_ROOT}/arduino-ide/.arduino15
mkdir -p "${ROS_WS_ROOT}/arduino-ide/Arduino IDE"
mkdir -p ${ROS_WS_ROOT}/arduino-ide/arduino-ide
mkdir -p ${ROS_WS_ROOT}/arduino-ide/Arduino
mkdir -p ${HOME}/.config

ln -s ${ROS_WS_ROOT}/arduino-ide/.arduinoIDE ${HOME}/.arduinoIDE
ln -s ${ROS_WS_ROOT}/arduino-ide/.arduino15 ${HOME}/.arduino15
ln -s "${ROS_WS_ROOT}/arduino-ide/Arduino IDE" "${HOME}/.config/Arduino IDE"
ln -s ${ROS_WS_ROOT}/arduino-ide/arduino-ide ${HOME}/arduino-ide
ln -s ${ROS_WS_ROOT}/arduino-ide/Arduino ${HOME}/Arduino
rm ${HOME}/.arduinoIDE/arduino-cli.yaml || true
ln -s /opt/${ORGANIZATION}/${PROJECT_NAME}/firmware/arduino_ide/arduino-cli.yaml ${HOME}/.arduinoIDE
