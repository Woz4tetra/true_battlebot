#!/usr/bin/env bash

LOG_PREFIX=${1:-log}
LOG_SUBDIR=$2

if [[ $TERM = "screen" ]] && [[ $(ps -p $PPID -o comm=) = tmux* ]]; then
    echo "Enabling tmux logging for ${LOG_PREFIX}"
    LOG_DIR=/data/logs/${LOG_SUBDIR}
    mkdir -p "${LOG_DIR}" 2> /dev/null
    LOG_NAME="${LOG_PREFIX}-$(date '+%Y-%m-%dT%H-%M-%S').log"
    LOG_PATH="${LOG_DIR}"/"${LOG_NAME}"
    rm "${LOG_DIR}"/latest 2> /dev/null
    ln -sf "${LOG_PATH}" "${LOG_DIR}"/latest
    script -f "${LOG_PATH}"
fi
