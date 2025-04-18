#!/bin/bash
echo "Starting ${PROJECT_NAME}"

SESSION=${PROJECT_NAME}

tmux has-session -t "${SESSION}" > /dev/null

if [ $? != 0 ]; then
    tmux new -s "${SESSION}" -d -x 200 -y 24
fi

APP_PID=/opt/"${ORGANIZATION}"/app_pid

echo 0 > ${APP_PID}

# SIGINT-handler
int_handler() {
    echo "Caught stop signal"
    pid=$(cat ${APP_PID})
    if [ "${pid}" -ne 0 ]; then
        echo "Stopping roslaunch"
        kill -SIGINT "${pid}"
        tail --pid="${pid}" -f /dev/null
    fi
    exit 0;
}
trap 'int_handler' SIGINT

wait_for_pid() {
    # Loop until the pid isn't zero
    while [ "$(cat "${APP_PID}")" -eq 0 ]; do
        sleep 1
    done
}

tmux send -t "${SESSION}" 'source /opt/${ORGANIZATION}/scripts/lib/enable_tmux_logger.sh ${PROJECT_NAME} ${PROJECT_NAME}' ENTER
tmux send -t "${SESSION}" 'source /opt/${ORGANIZATION}/scripts/lib/set_master.sh ${ROS_MASTER_INTERFACE}' ENTER
tmux send -t "${SESSION}" 'source /opt/${ORGANIZATION}/set_log_format.sh' ENTER
tmux send -t "${SESSION}" 'roslaunch --wait bw_bringup bw_bringup.launch --screen &' ENTER
tmux send -t "${SESSION}" 'echo $! > /opt/${ORGANIZATION}/app_pid' ENTER

wait_for_pid
pid=$(cat ${APP_PID})
tail --pid=$pid -f /dev/null
tmux kill-session -t "${SESSION}"

echo "${PROJECT_NAME} exited"
