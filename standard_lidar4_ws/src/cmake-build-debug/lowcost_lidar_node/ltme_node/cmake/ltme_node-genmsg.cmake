# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ltme_node: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ltme_node_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_custom_target(_ltme_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ltme_node" "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" ""
)

get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_custom_target(_ltme_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ltme_node" "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" ""
)

get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_custom_target(_ltme_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ltme_node" "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node
)
_generate_srv_cpp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node
)
_generate_srv_cpp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node
)

### Generating Module File
_generate_module_cpp(ltme_node
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ltme_node_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ltme_node_generate_messages ltme_node_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_cpp _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_cpp _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_cpp _ltme_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltme_node_gencpp)
add_dependencies(ltme_node_gencpp ltme_node_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltme_node_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node
)
_generate_srv_eus(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node
)
_generate_srv_eus(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node
)

### Generating Module File
_generate_module_eus(ltme_node
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ltme_node_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ltme_node_generate_messages ltme_node_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_eus _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_eus _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_eus _ltme_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltme_node_geneus)
add_dependencies(ltme_node_geneus ltme_node_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltme_node_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node
)
_generate_srv_lisp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node
)
_generate_srv_lisp(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node
)

### Generating Module File
_generate_module_lisp(ltme_node
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ltme_node_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ltme_node_generate_messages ltme_node_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_lisp _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_lisp _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_lisp _ltme_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltme_node_genlisp)
add_dependencies(ltme_node_genlisp ltme_node_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltme_node_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node
)
_generate_srv_nodejs(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node
)
_generate_srv_nodejs(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node
)

### Generating Module File
_generate_module_nodejs(ltme_node
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ltme_node_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ltme_node_generate_messages ltme_node_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_nodejs _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_nodejs _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_nodejs _ltme_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltme_node_gennodejs)
add_dependencies(ltme_node_gennodejs ltme_node_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltme_node_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node
)
_generate_srv_py(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node
)
_generate_srv_py(ltme_node
  "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node
)

### Generating Module File
_generate_module_py(ltme_node
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ltme_node_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ltme_node_generate_messages ltme_node_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QuerySerial.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_py _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryFirmwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_py _ltme_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/ltme_node/srv/QueryHardwareVersion.srv" NAME_WE)
add_dependencies(ltme_node_generate_messages_py _ltme_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ltme_node_genpy)
add_dependencies(ltme_node_genpy ltme_node_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ltme_node_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ltme_node
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ltme_node_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ltme_node
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ltme_node_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ltme_node
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ltme_node_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ltme_node
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ltme_node_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ltme_node
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ltme_node_generate_messages_py std_msgs_generate_messages_py)
endif()
