#!/bin/bash

SESSION=butler

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'datacentre'
tmux new-window -t $SESSION:2 -n 'bring_up'
tmux new-window -t $SESSION:3 -n 'navigation'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "source ~/bin/ros-setup.bash" C-m
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch --wait ros_datacentre datacentre.launch" C-m

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch --wait bham_b21_launch b21.launch chest_camera:=true" C-m

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch --wait bham_b21_2dnav b21_2d_nav.launch with_camera:=true  map:=/home/nah/strands_ws/src/butler/butler/maps/research_lab3.yaml" C-m 

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
