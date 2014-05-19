#!/bin/bash
SESSION=$USER

tmux -2 new-session -d -s $SESSION

# Setup a window for tailing log files
tmux new-window -t $SESSION:1 -n 'ROS'
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "sleep 15;bringup" C-m
tmux select-pane -t 1
tmux send-keys "sleep 50;ssh cob3-2-pc3 navi" C-m
tmux split-window -v
tmux resize-pane -D 20
tmux send-keys "roscore" C-m

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
