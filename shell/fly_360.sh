#!/bin/bash

# 启动第一个 roslaunch 命令在新窗口中运行
gnome-terminal --tab --title="sim.launch" -- bash -c "roslaunch tutorial_gazebo sim.launch; exec bash"

# 等待用户输入
read -p "Press Enter to start the second command..."

# 启动第二个 roslaunch 命令在另一个新窗口中运行
gnome-terminal --tab --title="run_craic_task.launch" -- bash -c "roslaunch tutorial_all run_craic_task.launch run_mavros:=false run_fast_lio:=false run_camera_driver:=false; exec bash"