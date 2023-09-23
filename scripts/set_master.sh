#!/usr/bin/env bash
INTERFACE=$1
HOST_MACHINE=`ifconfig ${INTERFACE} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`

if [[ $HOST_MACHINE =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
    export ROS_IP=${HOST_MACHINE}
    export ROS_MASTER_URI=http://${HOST_MACHINE}:11311
    # export ROS_MASTER_URI=http://${HOST_MACHINE_NAME}:11311

    echo ${ROS_IP}
    echo ${ROS_MASTER_URI}
else
    echo "Network not configured. Not setting ROS_IP or URI"
fi

