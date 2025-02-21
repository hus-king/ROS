cmake_minimum_required(VERSION 2.8.3)
project(image_capture_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  darknet_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_srvs darknet_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_capture_client src/image_capture_client.cpp)
target_link_libraries(image_capture_client ${catkin_LIBRARIES})