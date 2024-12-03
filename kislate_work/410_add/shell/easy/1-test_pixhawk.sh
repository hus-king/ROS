source /opt/ros/melodic/setup.bash
source ~/uav_ws/devel/setup.bash
#####test connection with px4#####
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"; exec bash"' \

