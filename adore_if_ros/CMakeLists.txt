#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Thomas Lobig - initial API and implementation
#********************************************************************************

cmake_minimum_required(VERSION 3.13)
project(adore_if_ros)

include(CTest)


include(../cmake/repo_url.config)#set the url of the git repo
include(../cmake/compile_options.config)#config compiler
include(../cmake/libadore.config)#download libadore
include(../cmake/plotlabserver.config)#download plotlabserver


set(CMAKE_MODULE_PATH
   ${CMAKE_CURRENT_SOURCE_DIR}/../external
#   ${CMAKE_CURRENT_SOURCE_DIR}/../libadore/external
   ${CMAKE_CURRENT_SOURCE_DIR}/../cmake
   CACHE INTERNAL "" FORCE
)


# the following lines should supress zmq test build
option(WITH_PERF_TOOL OFF)
option(ZMQ_BUILD_TESTS OFF)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED 
   COMPONENTS 
      roscpp
      std_msgs
      tf
      adore_if_ros_msg
)

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
#   std_msgs  # Or other packages containing msgs
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS adore_if_ros_msg
)

###########
## Build ##
###########


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/adore_if_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/adore_if_ros_node.cpp)
add_executable(adore_vehiclemodel_node)
target_sources(adore_vehiclemodel_node PRIVATE src/adore_vehiclemodel_node.cpp)
target_include_directories( adore_vehiclemodel_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_vehiclemodel_node PRIVATE ${catkin_LIBRARIES} adore::sim adore::apps adore::params)

add_executable(adore_objectdetectionmodel_node)
target_sources(adore_objectdetectionmodel_node PRIVATE src/adore_objectdetectionmodel_node.cpp)
target_include_directories( adore_objectdetectionmodel_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_objectdetectionmodel_node PRIVATE ${catkin_LIBRARIES} adore::sim adore::apps adore::params)

add_executable(adore_timer_node )
target_sources(adore_timer_node PRIVATE src/adore_timer_node.cpp)
target_include_directories(adore_timer_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_timer_node PRIVATE ${catkin_LIBRARIES} adore::sim)

add_executable(adore_feedbackcontroller_node )
target_sources(adore_feedbackcontroller_node PRIVATE src/adore_feedbackcontroller_node.cpp)
target_include_directories(adore_feedbackcontroller_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_feedbackcontroller_node PRIVATE ${catkin_LIBRARIES} adore::apps adore::params adore::fun adore::sim)

add_executable(adore_mapprovider_node)
target_sources(adore_mapprovider_node PRIVATE src/adore_mapprovider_node.cpp)
target_include_directories(adore_mapprovider_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_mapprovider_node PRIVATE ${catkin_LIBRARIES} adore::apps adore::env adore::params adore::if_xodr adore::sim)

add_executable(adore_navigation_node)
target_sources(adore_navigation_node PRIVATE src/adore_navigation_node.cpp)
target_include_directories(adore_navigation_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_navigation_node PRIVATE ${catkin_LIBRARIES} adore::apps adore::env adore::params adore::if_xodr)

add_executable(adore_borderbird_node)
target_sources(adore_borderbird_node PRIVATE src/adore_borderbird_node.cpp)
target_include_directories(adore_borderbird_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_borderbird_node PRIVATE ${catkin_LIBRARIES} adore::apps adore::params plotlablib adore::fun adore::sim)

add_executable(adore_lfbehavior_node src/adore_lfbehavior_node.cpp)
target_include_directories(adore_lfbehavior_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_lfbehavior_node PRIVATE ${catkin_LIBRARIES} adore::apps adore::fun adore::sim)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


   

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(
  TARGETS 
    adore_vehiclemodel_node
    adore_objectdetectionmodel_node
    adore_timer_node
    adore_feedbackcontroller_node
    adore_mapprovider_node
    adore_navigation_node
    adore_borderbird_node
    adore_lfbehavior_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_adore_if_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
