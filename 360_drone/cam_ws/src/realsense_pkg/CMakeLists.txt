cmake_minimum_required(VERSION 3.0.2)

project(realsense_pkg)


# Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS

  roscpp

)


# Find other required packages

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)


# Declare a catkin package

catkin_package(

  CATKIN_DEPENDS roscpp

)


# Specify additional locations of header files

include_directories(

  ${catkin_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS}

  ${realsense2_INCLUDE_DIRS}

)


# Declare the executable and specify its source files

add_executable(realsense_node src/find.cpp)


# Specify libraries to link the executable against

target_link_libraries(realsense_node

  ${catkin_LIBRARIES}

  ${OpenCV_LIBRARIES}

  ${realsense2_LIBRARY}

)
