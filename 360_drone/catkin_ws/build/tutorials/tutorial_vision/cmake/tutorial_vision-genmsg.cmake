# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tutorial_vision: 6 messages, 0 services")

set(MSG_I_FLAGS "-Itutorial_vision:/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tutorial_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" ""
)

get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" "std_msgs/Header:tutorial_vision/CircleInfo"
)

get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" "tutorial_vision/BoundingBox:std_msgs/Header"
)

get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_custom_target(_tutorial_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tutorial_vision" "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_cpp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(tutorial_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tutorial_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tutorial_vision_generate_messages tutorial_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_cpp _tutorial_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tutorial_vision_gencpp)
add_dependencies(tutorial_vision_gencpp tutorial_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tutorial_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)
_generate_msg_eus(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
)

### Generating Services

### Generating Module File
_generate_module_eus(tutorial_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tutorial_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tutorial_vision_generate_messages tutorial_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_eus _tutorial_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tutorial_vision_geneus)
add_dependencies(tutorial_vision_geneus tutorial_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tutorial_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)
_generate_msg_lisp(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(tutorial_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tutorial_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tutorial_vision_generate_messages tutorial_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_lisp _tutorial_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tutorial_vision_genlisp)
add_dependencies(tutorial_vision_genlisp tutorial_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tutorial_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)
_generate_msg_nodejs(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tutorial_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tutorial_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tutorial_vision_generate_messages tutorial_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_nodejs _tutorial_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tutorial_vision_gennodejs)
add_dependencies(tutorial_vision_gennodejs tutorial_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tutorial_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)
_generate_msg_py(tutorial_vision
  "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
)

### Generating Services

### Generating Module File
_generate_module_py(tutorial_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tutorial_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tutorial_vision_generate_messages tutorial_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleInfo.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/CircleDetectResult.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/StringStamped.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBox.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nx/catkin_ws/src/tutorials/tutorial_vision/msg/ObjectCount.msg" NAME_WE)
add_dependencies(tutorial_vision_generate_messages_py _tutorial_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tutorial_vision_genpy)
add_dependencies(tutorial_vision_genpy tutorial_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tutorial_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tutorial_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tutorial_vision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tutorial_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tutorial_vision_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tutorial_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tutorial_vision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tutorial_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tutorial_vision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tutorial_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tutorial_vision_generate_messages_py std_msgs_generate_messages_py)
endif()
