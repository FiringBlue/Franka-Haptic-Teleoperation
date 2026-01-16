# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "franka_teleop_lmt: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifranka_teleop_lmt:/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(franka_teleop_lmt_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_custom_target(_franka_teleop_lmt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_teleop_lmt" "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(franka_teleop_lmt
  "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_teleop_lmt
)

### Generating Services

### Generating Module File
_generate_module_cpp(franka_teleop_lmt
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_teleop_lmt
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(franka_teleop_lmt_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(franka_teleop_lmt_generate_messages franka_teleop_lmt_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_teleop_lmt_generate_messages_cpp _franka_teleop_lmt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_teleop_lmt_gencpp)
add_dependencies(franka_teleop_lmt_gencpp franka_teleop_lmt_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_teleop_lmt_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(franka_teleop_lmt
  "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_teleop_lmt
)

### Generating Services

### Generating Module File
_generate_module_eus(franka_teleop_lmt
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_teleop_lmt
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(franka_teleop_lmt_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(franka_teleop_lmt_generate_messages franka_teleop_lmt_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_teleop_lmt_generate_messages_eus _franka_teleop_lmt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_teleop_lmt_geneus)
add_dependencies(franka_teleop_lmt_geneus franka_teleop_lmt_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_teleop_lmt_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(franka_teleop_lmt
  "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_teleop_lmt
)

### Generating Services

### Generating Module File
_generate_module_lisp(franka_teleop_lmt
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_teleop_lmt
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(franka_teleop_lmt_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(franka_teleop_lmt_generate_messages franka_teleop_lmt_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_teleop_lmt_generate_messages_lisp _franka_teleop_lmt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_teleop_lmt_genlisp)
add_dependencies(franka_teleop_lmt_genlisp franka_teleop_lmt_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_teleop_lmt_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(franka_teleop_lmt
  "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_teleop_lmt
)

### Generating Services

### Generating Module File
_generate_module_nodejs(franka_teleop_lmt
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_teleop_lmt
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(franka_teleop_lmt_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(franka_teleop_lmt_generate_messages franka_teleop_lmt_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_teleop_lmt_generate_messages_nodejs _franka_teleop_lmt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_teleop_lmt_gennodejs)
add_dependencies(franka_teleop_lmt_gennodejs franka_teleop_lmt_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_teleop_lmt_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(franka_teleop_lmt
  "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_teleop_lmt
)

### Generating Services

### Generating Module File
_generate_module_py(franka_teleop_lmt
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_teleop_lmt
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(franka_teleop_lmt_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(franka_teleop_lmt_generate_messages franka_teleop_lmt_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_teleop_lmt_generate_messages_py _franka_teleop_lmt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_teleop_lmt_genpy)
add_dependencies(franka_teleop_lmt_genpy franka_teleop_lmt_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_teleop_lmt_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_teleop_lmt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_teleop_lmt
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_teleop_lmt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_teleop_lmt
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_teleop_lmt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_teleop_lmt
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_teleop_lmt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_teleop_lmt
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_teleop_lmt)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_teleop_lmt\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_teleop_lmt
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
