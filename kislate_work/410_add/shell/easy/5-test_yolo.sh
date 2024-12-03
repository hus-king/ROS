#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/uav_ws/devel/setup.bash
source ~/smart_ws/devel/setup.bash
#####test connection with yolo#####
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch darknet_ros darknet_ros.launch; exec bash"' \
