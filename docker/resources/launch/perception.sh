#!/bin/bash
echo "Starting ${PROJECT_NAME} perception"

SESSION=${PROJECT_NAME}_perception

tmux has-session -t "${SESSION}" > /dev/null

if [ $? != 0 ]; then
    tmux new -s "${SESSION}" -d -x 200 -y 24
fi

if [ ! -f /opt/"${ORGANIZATION}"/app_pid ]; then
    echo 0 > /opt/"${ORGANIZATION}"/app_pid
fi
# SIGINT-handler
int_handler() {
    echo "Caught stop signal"
    pid=$(cat /opt/"${ORGANIZATION}"/app_pid)
    if [ "${pid}" -ne 0 ]; then
        echo "Stopping perception"
        kill -SIGINT "${pid}"
        tail --pid="${pid}" -f /dev/null
    fi
    exit 0;
}
trap 'int_handler' SIGINT

tmux send -t "${SESSION}" 'source /opt/${ORGANIZATION}/scripts/lib/enable_tmux_logger.sh ${PROJECT_NAME}_perception perception' ENTER
tmux send -t "${SESSION}" 'source /opt/${ORGANIZATION}/scripts/lib/set_master.sh ${ROS_MASTER_INTERFACE}' ENTER
tmux send -t "${SESSION}" 'cd /opt/${ORGANIZATION}/${PROJECT_NAME}' ENTER
tmux send -t "${SESSION}" 'python perception/detection/main.py perception/configs &' ENTER
tmux send -t "${SESSION}" 'echo $! > /opt/${ORGANIZATION}/app_pid' ENTER

sleep infinity
