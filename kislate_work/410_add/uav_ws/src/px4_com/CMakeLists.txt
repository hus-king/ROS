cmake_minimum_required(VERSION 2.8.3)
project(px4_command)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  message_generation
  darknet_ros_msgs
  image_transport
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  command.msg
  ude_log.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################

###### Main File ##########
add_executable(px4_pos_controller
  src/px4_pos_controller.cpp
)
add_dependencies(px4_pos_controller px4_command_gencpp)
target_link_libraries(px4_pos_controller
  ${catkin_LIBRARIES}
)

##
add_executable(px4_pos_estimator
  src/px4_pos_estimator.cpp
)
add_dependencies(px4_pos_estimator px4_command_gencpp)
target_link_libraries(px4_pos_estimator
  ${catkin_LIBRARIES}
)

###### pose gesture control ##########
add_executable(px4_gesture_control
  src/px4_gesture_control.cpp
)
add_dependencies(px4_gesture_control px4_command_gencpp)
target_link_libraries(px4_gesture_control
  ${catkin_LIBRARIES}
)


###### Utilities File ##########
add_executable(TFmini
  src/Utilities/TFmini.cpp
)
add_dependencies(TFmini px4_command_gencpp)
target_link_libraries(TFmini
  ${catkin_LIBRARIES}
)

###### Application File ##########


#add_executable(mission_cys
#  src/Application/mission_cys.cpp
#)
#add_dependencies(mission_cys px4_command_gencpp)
#target_link_libraries(mission_cys
#  ${catkin_LIBRARIES}
#)

#add_executable(mission_zgjqrds
 # src/Application/mission_zgjqrds.cpp
#)
#add_dependencies(mission_zgjqrds px4_command_gencpp)
#target_link_libraries(mission_zgjqrds
 # ${catkin_LIBRARIES}
#)

add_executable(auto_takeoff
  src/Application/auto_takeoff.cpp
)
add_dependencies(auto_takeoff px4_command_gencpp)
target_link_libraries(auto_takeoff
  ${catkin_LIBRARIES}
)

add_executable(square
  src/Application/square.cpp
)
add_dependencies(square px4_command_gencpp)
target_link_libraries(square
  ${catkin_LIBRARIES}
)

add_executable(collision_avoidance
  src/Application/collision_avoidance.cpp
)
add_dependencies(collision_avoidance px4_command_gencpp)
target_link_libraries(collision_avoidance
  ${catkin_LIBRARIES}
)

add_executable(vision_track
  src/Application/vision_track.cpp
)
add_dependencies(vision_track px4_command_gencpp)
target_link_libraries(vision_track
  ${catkin_LIBRARIES}
)

add_executable(ABCdemo
  src/Application/4x4ABCdemo.cpp
)
add_dependencies(ABCdemo px4_command_gencpp)
target_link_libraries(ABCdemo
  ${catkin_LIBRARIES}
)

add_executablecrossing_door_darknet
  src/Application/crossing_door_darknet.cpp
)
add_dependenciescrossing_door_darknet px4_command_gencpp)
target_link_librariescrossing_door_darknet
  ${catkin_LIBRARIES}
)


add_executable(deliver_goods
  src/Application/deliver_goods.cpp
)
add_dependencies(deliver_goods px4_command_gencpp)
target_link_libraries(deliver_goods
  ${catkin_LIBRARIES}
)

add_executable(uav_cruise
  src/Application/uav_cruise.cpp
)
add_dependencies(uav_cruise px4_command_gencpp)
target_link_libraries(uav_cruise
  ${catkin_LIBRARIES}
)

add_executable(px4_vel_controller
  src/px4_vel_controller.cpp
)
add_dependencies(px4_vel_controller px4_command_gencpp)
target_link_libraries(px4_vel_controller
  ${catkin_LIBRARIES}
)
#add_executable(crossing_door_darknet
#  src/Application/crossing_door_darknet.cpp
#)
#add_dependencies(crossing_door_darknet px4_command_gencpp)
#target_link_libraries(crossing_door_darknet
#  ${catkin_LIBRARIES}
#)

#add_executable(firematch
#  src/Application/firematch.cpp
#)
#add_dependencies(firematch px4_command_gencpp)
#target_link_libraries(firematch
#  ${catkin_LIBRARIES}
#)

#add_executable(flytwopoints
#  src/Application/flytwopoints.cpp
#)
#add_dependencies(flytwopoints px4_command_gencpp)
#target_link_libraries(flytwopoints
#  ${catkin_LIBRARIES}
#)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_px4_command.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
