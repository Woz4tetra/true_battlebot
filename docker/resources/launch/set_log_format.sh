#!/bin/bash
export ROSCONSOLE_FORMAT='{"severity": "${severity}", "message": "<-<${message}>->", "time": ${time}, "walltime": ${walltime}, "thread": "${thread}", "logger": "${logger}", "file": "${file}", "line": ${line}, "function": "${function}", "node": "${node}"}'
export ROSCONSOLE_CONFIG_FILE=/opt/${ORGANIZATION}/rosconsole.config
