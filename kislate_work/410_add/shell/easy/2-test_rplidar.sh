source /opt/ros/melodic/setup.bash
source ~/uav_ws/devel/setup.bash
#####test connection with rplidar#####
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch rplidar_ros view_rplidar.launch; exec bash"' \
