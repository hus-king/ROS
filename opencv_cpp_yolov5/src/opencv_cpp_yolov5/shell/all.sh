#!/bin/bash

# 创建一个新的tmux会话
SESSION_NAME="vision"
tmux new-session -d -s $SESSION_NAME

# 窗口1: roslaunch opencv_cpp_yolov5 usb_cam.launch
tmux rename-window -t 0 'USB Cam'
tmux send-keys -t 'USB Cam' 'roslaunch opencv_cpp_yolov5 usb_cam.launch' C-m

# 窗口2: roslaunch rosbridge_server rosbridge_websocket.launch
tmux new-window -t $SESSION_NAME -n 'Rosbridge'
tmux send-keys -t 'Rosbridge' 'sleep 5' C-m
tmux send-keys -t 'Rosbridge' 'roslaunch rosbridge_server rosbridge_websocket.launch' C-m

# 窗口3: rosrun web_video_server web_video_server
tmux new-window -t $SESSION_NAME -n 'Web Video Server'
tmux send-keys -t 'Web Video Server' 'sleep 5' C-m
tmux send-keys -t 'Web Video Server' 'rosrun web_video_server web_video_server' C-m

# 窗口4: roslaunch opencv_cpp_yolov5 opencv_cpp_yolov5.launch
tmux new-window -t $SESSION_NAME -n 'YOLOv5'
tmux send-keys -t 'YOLOv5' 'sleep 5' C-m
tmux send-keys -t 'YOLOv5' 'roslaunch opencv_cpp_yolov5 opencv_cpp_yolov5.launch' C-m

# 窗口5: roslaunch opencv_cpp_yolov5 hough_circle_detector.launch
tmux new-window -t $SESSION_NAME -n 'Hough Circle Detector'
tmux send-keys -t 'Hough Circle Detector' 'sleep 5' C-m
tmux send-keys -t 'Hough Circle Detector' 'roslaunch opencv_cpp_yolov5 hough_circle_detector.launch' C-m

# 返回到第一个窗口
tmux select-window -t $SESSION_NAME:0

# 附加到会话
tmux attach-session -t $SESSION_NAME