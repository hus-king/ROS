source /opt/ros/melodic/setup.bash
#####test connection with usb-cam#####
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
--tab -e  'bash -c "sleep 7; rosrun image_view image_view image:=/usb_cam/image_raw; exec bash"' \
