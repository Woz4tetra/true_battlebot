#!/usr/bin/env bash

LOG_PREFIX=${1:-log}
LOG_SUBDIR=${2:-}

LOG_DIR=/media/storage/logs/${LOG_SUBDIR}
LATEST_LINK=${LOG_DIR}/latest
if [ -L ${LATEST_LINK} ]; then
    echo "Cleaning up tmux logging for ${LOG_PREFIX}"
    LATEST_LOG=$(realpath ${LATEST_LINK})
    rm ${LOG_DIR}/last > /dev/null 2>&1
    ln -s ${LATEST_LOG} ${LOG_DIR}/last
    rm ${LOG_DIR}/latest> /dev/null 2>&1
fi
