#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")

source ${BASE_DIR}/set_client.sh $1
roslaunch bw_joystick bw_joystick.launch topic_name:=joy_remote device:=$2
