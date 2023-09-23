#!/bin/bash
echo "Starting ${PROJECT_NAME}"

SESSION=${PROJECT_NAME}

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

if [ ! -f /opt/${ORGANIZATION}/roslaunch_pid ]; then
    echo 0 > /opt/${ORGANIZATION}/roslaunch_pid
fi
# SIGINT-handler
int_handler() {
    echo "Caught stop signal"
    pid=`cat /opt/${ORGANIZATION}/roslaunch_pid`
    if [ $pid -ne 0 ]; then
        echo "Stopping roslaunch"
        kill -SIGINT $pid
        tail --pid=$pid -f /dev/null
    fi
    exit 0;
}
trap 'int_handler' SIGINT

tmux send -t $SESSION 'source /opt/${ORGANIZATION}/scripts/enable_tmux_logger.sh ${PROJECT_NAME}' ENTER
tmux send -t $SESSION 'source /opt/${ORGANIZATION}/scripts/set_master.sh ${ROS_MASTER_INTERFACE}' ENTER
tmux send -t $SESSION 'source /opt/${ORGANIZATION}/entrypoint.sh' ENTER
tmux send -t $SESSION 'roslaunch --wait bw_bringup bw_bringup.launch --screen &' ENTER
tmux send -t $SESSION 'echo $! > /opt/${ORGANIZATION}/roslaunch_pid' ENTER

sleep 2
/opt/${ORGANIZATION}/scripts/logs
sleep infinity
