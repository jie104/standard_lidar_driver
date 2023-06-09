cmake_minimum_required(VERSION 2.8.3)
project(standard_lidar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
#  liburg_c
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES standard_lidar_driver
  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3" # error2
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/standard_lidar_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lidar_driver_node.cpp)
# set(URG_SRC PROJECT_SOURCE_DIR/src/standard_lidar_driver/src/standard_lidar_protocol/UST10LX/urg_c/src)
# file(GLOB_RECURSE URG_file "URG_SRC/*.c")
# add_library(urgc STATIC ${URG_file})
#include_directories(${LIBRARY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(liburg_c  src/standard_lidar_protocol/UST10LX/urg_c/src/urg_sensor.c
		src/standard_lidar_protocol/UST10LX/urg_c//src/urg_utils.c
		src/standard_lidar_protocol/UST10LX/urg_c//src/urg_debug.c
		src/standard_lidar_protocol/UST10LX/urg_c//src/urg_connection.c
		src/standard_lidar_protocol/UST10LX/urg_c//src/urg_ring_buffer.c
		src/standard_lidar_protocol/UST10LX/urg_c/src/urg_serial_linux.c
		src/standard_lidar_protocol/UST10LX/urg_c/src//urg_serial_utils_linux.c
		src/standard_lidar_protocol/UST10LX/urg_c//src/urg_tcpclient.c
		src/standard_lidar_protocol/UST10LX/urg_c/src/urg_time.c

)

add_library(urg_c_wrapper  src/standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp)


add_executable(simi_node src/simi_nodes/lidar_driver_simi.cpp)
add_executable(simi_node_1 src/simi_nodes/lidar_driver_simi1.cpp)
add_executable(simi_node_2 src/simi_nodes/lidar_driver_simi2.cpp)


#add_executable(wanji_node src/lidar_driver_wanji.cpp)
add_executable(oradar_node src/oradar_nodes/lidar_driver_oradar.cpp)
add_executable(oradar_node_1 src/oradar_nodes/lidar_driver_oradar1.cpp)
add_executable(oradar_node_2 src/oradar_nodes/lidar_driver_oradar2.cpp)

add_executable(hokuyo_node src/lidar_driver_hokuyo.cpp)

add_executable(free_optics_node src/free_optics.cpp)
add_executable(precision_anaysis_node src/data_anaysis/precision_anaysis.cpp)

add_library(data_anaysis SHARED src/data_anaysis/data_anaysis.cpp)


#add_executable(oba_keli_node src/lidar_driver_oba_keli.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#  target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#          glog
#  )
target_link_libraries(simi_node
        ${catkin_LIBRARIES}
        glog
        )
target_link_libraries(simi_node_1
		${catkin_LIBRARIES}
		glog
		)

target_link_libraries(simi_node_2
		${catkin_LIBRARIES}
		glog
		)





#target_link_libraries(wanji_node
#        ${catkin_LIBRARIES}
#        glog)

target_link_libraries(oradar_node
${catkin_LIBRARIES}
glog)

target_link_libraries(oradar_node_1
		${catkin_LIBRARIES}
		glog)

target_link_libraries(oradar_node_2
		${catkin_LIBRARIES}
		glog)


target_link_libraries(hokuyo_node PUBLIC
${catkin_LIBRARIES}
urg_c_wrapper
glog
liburg_c
)

target_link_libraries(free_optics_node
		${catkin_LIBRARIES}
		glog)

target_link_libraries(precision_anaysis_node
		${catkin_LIBRARIES}
		data_anaysis
		glog)



#target_link_libraries(oba_keli_node
#		${catkin_LIBRARIES}
#		glog)


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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_standard_lidar_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
