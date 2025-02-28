#!/bin/bash

# Create a new tmux session
tmux new-session -d -s zal_yolo_session

# Run commands in separate tmux panes with delay
tmux send-keys -t zal_yolo_session 'roslaunch rosbridge_server rosbridge_websocket.launch' C-m
sleep 5

tmux split-window -v -t zal_yolo_session
tmux send-keys -t zal_yolo_session 'rosrun web_video_server web_video_server' C-m
sleep 5

tmux split-window -h -t zal_yolo_session
tmux send-keys -t zal_yolo_session 'roslaunch tutorial_vision simple_camera_driver.launch' C-m
sleep 5

tmux select-pane -U
tmux split-window -h -t zal_yolo_session
tmux send-keys -t zal_yolo_session 'roslaunch tutorial_vision zal_yolov5.launch' C-m

# Attach to the tmux session
tmux attach-session -t zal_yolo_session