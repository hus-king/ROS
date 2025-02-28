cmake_minimum_required(VERSION 3.0.2)

project(point_depth_viewer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  camera_processor
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

## Uncomment this if the package has a setup.py
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_depth_viewer
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(point_depth_viewer_node src/point_depth_viewer.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(point_depth_viewer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(point_depth_viewer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
