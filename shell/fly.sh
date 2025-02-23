#!/bin/sh
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo

gnome-terminal --window -e 'bash -c "source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo; roslaunch simulation iris_rplidar.launch; exec bash"' \
--tab -e 'bash -c "cd ~/Desktop/; ./QGroundControl.AppImage; exec bash"' \
--tab -e 'bash -c "cd ~/410_shell/middle/; ./collision_avoidance.sh; exec bash"'
