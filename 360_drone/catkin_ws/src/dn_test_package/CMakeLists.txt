cmake_minimum_required(VERSION 3.1)

project(dn_test_package)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  darknet_ros_msgs
  camera_processor
  message_generation
)

find_package(OpenCV 4.5.4 REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dn_test src/dn_test.cpp)

target_link_libraries(dn_test ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
