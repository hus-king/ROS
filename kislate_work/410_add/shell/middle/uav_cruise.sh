source /opt/ros/melodic/setup.bash
source ~/uav_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_vel_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command qr_code_scan.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command uav_cruise.launch; exec bash"' \
