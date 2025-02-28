cmake_minimum_required(VERSION 3.0.2)
project(depth_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  camera_processor
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(depth_visualizer_node src/depth_visualizer.cpp)
target_link_libraries(depth_visualizer_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
