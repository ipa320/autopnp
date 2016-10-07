# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ros_system: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ros_system_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/SubstractTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_system
)
_generate_srv_cpp(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_system
)

### Generating Module File
_generate_module_cpp(ros_system
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_system
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ros_system_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ros_system_generate_messages ros_system_generate_messages_cpp)

# target for backward compatibility
add_custom_target(ros_system_gencpp)
add_dependencies(ros_system_gencpp ros_system_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_system_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/SubstractTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_system
)
_generate_srv_lisp(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_system
)

### Generating Module File
_generate_module_lisp(ros_system
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_system
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ros_system_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ros_system_generate_messages ros_system_generate_messages_lisp)

# target for backward compatibility
add_custom_target(ros_system_genlisp)
add_dependencies(ros_system_genlisp ros_system_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_system_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/SubstractTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system
)
_generate_srv_py(ros_system
  "/home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system
)

### Generating Module File
_generate_module_py(ros_system
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ros_system_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ros_system_generate_messages ros_system_generate_messages_py)

# target for backward compatibility
add_custom_target(ros_system_genpy)
add_dependencies(ros_system_genpy ros_system_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_system_generate_messages_py)


debug_message(2 "ros_system: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_system
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ros_system_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_system
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ros_system_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_system
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ros_system_generate_messages_py std_msgs_generate_messages_py)
