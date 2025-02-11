#!/bin/bash

# 创建一个新的tmux会话
SESSION_NAME="vision"
tmux new-session -d -s $SESSION_NAME

# 窗口1: roslaunch opencv_cpp_yolov5 usb_cam.launch（USB Cam）：启动 usb_cam.launch 文件，用于摄像头数据采集。
tmux rename-window -t 0 'USB Cam'
tmux send-keys -t 'USB Cam' 'roslaunch opencv_cpp_yolov5 usb_cam.launch' C-m

# 窗口2: roslaunch rosbridge_server rosbridge_websocket.launch，用于 ROS 与外部通信（如网页端）。
tmux new-window -t $SESSION_NAME -n 'Rosbridge'
tmux send-keys -t 'Rosbridge' 'sleep 5' C-m
tmux send-keys -t 'Rosbridge' 'roslaunch rosbridge_server rosbridge_websocket.launch' C-m

# 窗口3: rosrun web_video_server web_video_server，用于将 ROS 中的视频流通过 HTTP 提供给网页端。
tmux new-window -t $SESSION_NAME -n 'Web Video Server'
tmux send-keys -t 'Web Video Server' 'sleep 5' C-m
tmux send-keys -t 'Web Video Server' 'rosrun web_video_server web_video_server' C-m

# 窗口4: roslaunch opencv_cpp_yolov5 opencv_cpp_yolov5.launch，用于运行 YOLOv5 目标检测。
tmux new-window -t $SESSION_NAME -n 'YOLOv5'
tmux send-keys -t 'YOLOv5' 'sleep 5' C-m
tmux send-keys -t 'YOLOv5' 'roslaunch opencv_cpp_yolov5 opencv_cpp_yolov5.launch' C-m

# 窗口5: roslaunch opencv_cpp_yolov5 hough_circle_detector.launch，用于圆检测。
tmux new-window -t $SESSION_NAME -n 'Hough Circle Detector'
tmux send-keys -t 'Hough Circle Detector' 'sleep 5' C-m
tmux send-keys -t 'Hough Circle Detector' 'roslaunch opencv_cpp_yolov5 hough_circle_detector.launch' C-m

# 返回到第一个窗口
tmux select-window -t $SESSION_NAME:0

# 附加到会话
tmux attach-session -t $SESSION_NAME