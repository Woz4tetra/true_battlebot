#!/usr/bin/env bash
set -e

INTERFACE="${1}"

HOST_MACHINE=$(hostname -I | awk '{print $1}')
if [ -n "${INTERFACE}" ]; then
    INTERFACE_IP=$(ifconfig "${INTERFACE}" | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')

    if [[ $INTERFACE_IP =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
        HOST_MACHINE=${INTERFACE_IP}
    fi
fi

export ROS_IP=${HOST_MACHINE}
export ROS_MASTER_URI=http://${HOST_MACHINE}:11311
echo "ROS IP ${ROS_IP}"
