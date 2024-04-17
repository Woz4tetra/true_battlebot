#!/bin/bash
tmux new -s train -d
tmux send -t train "cd ~/true_battlebot/training && python train.py" ENTER
tmux a -t train
