#!/usr/bin/env bash

LOGPREFIX=${1:-log}

if [[ $TERM = "screen" ]] && [[ $(ps -p $PPID -o comm=) = tmux* ]]; then
    echo "Enabling tmux logging for $LOGPREFIX"
    LOGDIR=/media/storage/logs
    mkdir $LOGDIR 2> /dev/null
    LOGNAME="$LOGPREFIX-$(date '+%Y-%m-%dT%H-%M-%S').log"
    script -f $LOGDIR/${LOGNAME}
fi
