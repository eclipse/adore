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
enable_testing()

#option() does nothing if option already defined
option(BUILD_adore_DEBUG_BUILD_PROCESS OFF)

if(BUILD_adore_DEBUG_BUILD_PROCESS)
  add_compile_options(-save-temps=obj)
endif()

#option() does nothing if option already defined
option(BUILD_adore_with_stacktrace OFF)

if(BUILD_adore_with_stacktrace)
  add_compile_options(-g1 -lSegFault)
endif()



cmake_policy(SET CMP0048 NEW)
add_compile_options("-fdiagnostics-color=always")
#set(USE_AVX_INSTRUCTIONS "1")
#set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -fwhole-program -DNDEBUG")

#MESSAGE(NOTICE "Build type: " ${CMAKE_BUILD_TYPE})
#MESSAGE(NOTICE "Library Type: " ${LIB_TYPE})
#MESSAGE(NOTICE "Compiler flags:" ${CMAKE_CXX_COMPILE_FLAGS})
#MESSAGE(NOTICE "Compiler cxx debug flags:" ${CMAKE_CXX_FLAGS_DEBUG})
#MESSAGE(NOTICE "Compiler cxx release flags:" ${CMAKE_CXX_FLAGS_RELEASE})
#MESSAGE(NOTICE "Compiler cxx min size flags:" ${CMAKE_CXX_FLAGS_MINSIZEREL})
#MESSAGE(NOTICE "Compiler cxx flags:" ${CMAKE_CXX_FLAGS})


include(GNUInstallDirs)

list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../adore_if_ros_msg/adore_if_ros_msg/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../plotlablib/plotlablib/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../libadore/libadore/build/install")

add_library(qpoases::qpoases STATIC IMPORTED GLOBAL)
set_property(TARGET qpoases::qpoases PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../libadore/libadore/build/install/lib/libqpOASES.a)
#include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../external/qpOASES)


#add_library(dlib::dlib STATIC IMPORTED GLOBAL)
#set_property(TARGET dlib::dlib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../external/dlib/build/dlib/libdlib.a)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_FIND_DEBUG_MODE TRUE)
#find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)
#find_program(CMAKE_CXX_CPPLINT NAMES cpplint)
if (CMAKE_CXX_CPPCHECK)
    list(
        APPEND CMAKE_CXX_CPPCHECK 
            "--enable=all"
            "--output-file=cppcheck.log"
            "--inconclusive"
            "--force" 
            "--inline-suppr"
            "--error-exitcode=1"
    )
endif()
#            "--suppressions-list=${CMAKE_SOURCE_DIR}/CppCheckSuppressions.txt"

find_package(adore_if_ros_msg CONFIG REQUIRED)
find_package(xodr CONFIG REQUIRED)
find_package(csaps-cpp CONFIG REQUIRED)

find_package(dlib CONFIG REQUIRED)
#find_package(qpOASES CONFIG REQUIRED)
find_package(params CONFIG REQUIRED)
find_package(view CONFIG REQUIRED)
find_package(mad CONFIG REQUIRED)
find_package(fun CONFIG REQUIRED)
find_package(env CONFIG REQUIRED)
find_package(if_xodr CONFIG REQUIRED)
find_package(if_r2s CONFIG REQUIRED)
find_package(sim CONFIG REQUIRED)
find_package(apps CONFIG REQUIRED)

find_package(plotlablib CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(ZeroMQ CONFIG REQUIRED)

#find_package (xsdcxx REQUIRED)
find_package(XercesC 3.2.0 REQUIRED)


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
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ../../adore_if_ros_msg/adore_if_ros_msg)
#message(WARNING ${CMAKE_PREFIX_PATH})
#find_package(
#      adore_if_ros_msg CONFIG
#)

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
  CATKIN_DEPENDS 
    tf
    adore_if_ros_msg
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

## use this line to add a stack trace to your app
# target_compile_options(adore_TARGETNAME_node BEFORE PRIVATE -g1 -lSegFault)

message(${catkin_INCLUDE_DIRS})

add_executable(adore_vehiclemodel_node)
target_sources(adore_vehiclemodel_node PRIVATE src/adore_vehiclemodel_node.cpp)
target_include_directories( 
    adore_vehiclemodel_node 
      PRIVATE include 
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
    adore_vehiclemodel_node 
      PRIVATE 
        ${catkin_LIBRARIES} 
        libadore::sim 
        libadore::apps 
        libadore::params
)

add_executable(adore_odometrymodel_node)
target_sources(adore_odometrymodel_node PRIVATE src/adore_odometrymodel_node.cpp)
target_include_directories( adore_odometrymodel_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_odometrymodel_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_localizationmodel_node)
target_sources(adore_localizationmodel_node PRIVATE src/adore_localizationmodel_node.cpp)
target_include_directories( adore_localizationmodel_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_localizationmodel_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::sim 
                          libadore::apps 
                          libadore::params 
                          dlib::dlib
)

add_executable(adore_objectdetectionmodel_node)
target_sources(adore_objectdetectionmodel_node PRIVATE src/adore_objectdetectionmodel_node.cpp)
target_include_directories(
    adore_objectdetectionmodel_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_objectdetectionmodel_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_timer_node )
target_sources(adore_timer_node PRIVATE src/adore_timer_node.cpp)
target_include_directories(adore_timer_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_timer_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::sim
)

add_executable(adore_scheduler_node )
target_sources(adore_scheduler_node PRIVATE src/adore_scheduler_node.cpp)
target_include_directories(adore_scheduler_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_scheduler_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::sim 
                          dlib::dlib
)

add_executable(adore_feedbackcontroller_node )
target_sources(adore_feedbackcontroller_node PRIVATE src/adore_feedbackcontroller_node.cpp)
target_include_directories(adore_feedbackcontroller_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_feedbackcontroller_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          libadore::sim 
)

add_executable(adore_mapprovider_node)
target_sources(adore_mapprovider_node PRIVATE src/adore_mapprovider_node.cpp)
target_include_directories(adore_mapprovider_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_mapprovider_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::env 
                          libadore::params 
                          libadore::if_xodr 
                          libadore::sim
                          libadore::if_r2s 
                          plotlablib::plotlablib
)

add_executable(adore_speedlimitprovider_node)
target_sources(adore_speedlimitprovider_node PRIVATE src/adore_speedlimitprovider_node.cpp)
target_include_directories(adore_speedlimitprovider_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_speedlimitprovider_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::env 
                          libadore::params 
                          libadore::sim
                          plotlablib::plotlablib
)

add_executable(adore_navigation_node)
target_sources(adore_navigation_node PRIVATE src/adore_navigation_node.cpp)
target_include_directories(adore_navigation_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_navigation_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::env 
                          libadore::params 
                          libadore::if_xodr
                          libadore::if_r2s
                          plotlablib::plotlablib
)

add_executable(adore_indicatorhintprovider_node)
target_sources(
    adore_indicatorhintprovider_node 
      PRIVATE 
        src/adore_indicatorhintprovider_node.cpp
)
target_include_directories(
    adore_indicatorhintprovider_node 
      PRIVATE 
        include 
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(adore_indicatorhintprovider_node 
                        PRIVATE 
                          ${catkin_LIBRARIES}
                          plotlablib::plotlablib
                          libadore::apps
                          libadore::env
                          libadore::params
                          libadore::sim
)

# add_executable(adore_borderbird_node)
# target_sources(adore_borderbird_node PRIVATE src/adore_borderbird_node.cpp)
# target_include_directories(adore_borderbird_node PRIVATE include ${catkin_INCLUDE_DIRS})
# target_link_libraries(adore_borderbird_node PRIVATE ${catkin_LIBRARIES} libadore::apps libadore::params libadore::fun libadore::sim)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
add_executable(plot_area_of_effect)
target_sources(plot_area_of_effect PRIVATE src/plot_area_of_effect.cpp)
target_include_directories(
    plot_area_of_effect 
      PRIVATE 
        include
        "$<TARGET_PROPERTY:plotlablib::plotlablib,INTERFACE_INCLUDE_DIRECTORIES>"
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
    plot_area_of_effect 
      PRIVATE 
        ${catkin_LIBRARIES} 
        plotlablib::plotlablib
        libadore::env 
)

add_executable(plot_trajectories_node)
target_sources(plot_trajectories_node PRIVATE src/plot_trajectories_node.cpp)
target_include_directories(plot_trajectories_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_trajectories_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          plotlablib::plotlablib
                          libadore::fun 
)

add_executable(plot_planning_details_node)
target_sources(plot_planning_details_node PRIVATE src/plot_planning_details_node.cpp)
target_include_directories(plot_planning_details_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_planning_details_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          plotlablib::plotlablib
                          libadore::apps 
                          libadore::mad 
                          libadore::fun 
)

add_executable(plot_graph_node)
target_sources(plot_graph_node PRIVATE src/plot_graph_node.cpp)
target_include_directories(plot_graph_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_graph_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          plotlablib::plotlablib
)

add_executable(test_trajectory_planner_node src/test_trajectory_planner_node.cpp)
target_include_directories(test_trajectory_planner_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_trajectory_planner_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          dlib::dlib
                          plotlablib::plotlablib 
                          libadore::apps 
                          libadore::fun 
                          libadore::sim 
)

add_executable(test_mrm_planner_node src/test_mrm_planner_node.cpp)
target_include_directories(test_mrm_planner_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_mrm_planner_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::params 
                          libadore::sim
                          plotlablib::plotlablib
)

add_executable(test_straight_line_prediction_node)
target_sources(test_straight_line_prediction_node PRIVATE src/test_straight_line_prediction_node.cpp)
target_include_directories( test_straight_line_prediction_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_straight_line_prediction_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_gap_provider)
target_sources(adore_gap_provider PRIVATE src/adore_gap_provider.cpp)
target_include_directories( adore_gap_provider PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_gap_provider 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params 
                          dlib::dlib
)

add_executable(adore_prediction_provider)
target_sources(adore_prediction_provider PRIVATE src/adore_prediction_provider.cpp)
target_include_directories( adore_prediction_provider PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_prediction_provider 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_prediction_filter)
target_sources(adore_prediction_filter PRIVATE src/adore_prediction_filter.cpp)
target_include_directories( adore_prediction_filter PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_prediction_filter 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::mad 
                          libadore::apps 
                          libadore::params 
                          libadore::sim
)

add_executable(adore_lvprovider_node)
target_sources(adore_lvprovider_node PRIVATE src/adore_lvprovider_node.cpp)
target_include_directories( adore_lvprovider_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_lvprovider_node 
                        PRIVATE 
                          ${catkin_LIBRARIES}
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_crosstraffic_provider)
target_sources(adore_crosstraffic_provider PRIVATE src/adore_crosstraffic_provider.cpp)
target_include_directories( adore_crosstraffic_provider PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_crosstraffic_provider 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(adore_checkpoint_controller_node)
target_sources(adore_checkpoint_controller_node PRIVATE src/adore_checkpoint_controller_node.cpp)
target_include_directories( adore_checkpoint_controller_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_checkpoint_controller_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::mad 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params 
                          dlib::dlib
)

add_executable(adore_mission_controller_node)
target_sources(adore_mission_controller_node PRIVATE src/adore_mission_controller_node.cpp)
target_include_directories( adore_mission_controller_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_mission_controller_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::mad 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)

add_executable(test_control_dashboard src/test_control_dashboard.cpp)
target_include_directories(test_control_dashboard PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_control_dashboard 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::params
                          plotlablib::plotlablib
)

add_executable(test_control_error_node src/test_control_error_node.cpp)
target_include_directories(
    test_control_error_node 
      PUBLIC 
        include 
        "$<TARGET_PROPERTY:plotlablib::plotlablib,INTERFACE_INCLUDE_DIRECTORIES>"
        ${catkin_INCLUDE_DIRS}
        ${adore_if_ros_msg_INCLUDE_DIRS}
)
target_link_libraries(test_control_error_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::params
)

add_executable(plot_satimages_node)
target_sources(plot_satimages_node PRIVATE src/plot_satimages_node.cpp)
target_include_directories(plot_satimages_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_satimages_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_ego_node)
target_sources(plot_ego_node PRIVATE src/plot_ego_node.cpp)
target_include_directories(plot_ego_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_ego_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_gaps_node)
target_sources(plot_gaps_node PRIVATE src/plot_gaps.cpp)
target_include_directories(plot_gaps_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_gaps_node 
                        PRIVATE 
                        ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::env 
                          plotlablib::plotlablib
)

add_executable(plot_lanes_node)
target_sources(plot_lanes_node PRIVATE src/plot_lanes_node.cpp)
target_include_directories(plot_lanes_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_lanes_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params  
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_scalar_node)
target_sources(plot_scalar_node PRIVATE src/plot_scalar_node.cpp)
target_include_directories(plot_scalar_node PRIVATE include ${catkin_INCLUDE_DIRS} )
target_link_libraries(plot_scalar_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_vector_node)
target_sources(plot_vector_node PRIVATE src/plot_vector_node.cpp)
target_include_directories(plot_vector_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_vector_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_traffic_node)
target_sources(plot_traffic_node PRIVATE src/plot_traffic_node.cpp)
target_include_directories(plot_traffic_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_traffic_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib 
                          dlib::dlib
)

add_executable(plot_trafficlights_node)
target_sources(plot_trafficlights_node PRIVATE src/plot_trafficlights_node.cpp)
target_include_directories(plot_trafficlights_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_trafficlights_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_predictions_node)
target_sources(plot_predictions_node PRIVATE src/plot_predictions_node.cpp)
target_include_directories(plot_predictions_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_predictions_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_predictions_minimal_node)
target_sources(plot_predictions_minimal_node PRIVATE src/plot_predictions_minimal_node.cpp)
target_include_directories(plot_predictions_minimal_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_predictions_minimal_node 
                        PRIVATE
                          ${catkin_LIBRARIES}
                          plotlablib::plotlablib
                          libadore::apps
                          libadore::params
                          libadore::fun
)

add_executable(plot_conflicts_node)
target_sources(plot_conflicts_node PRIVATE src/plot_conflicts_node.cpp)
target_include_directories(plot_conflicts_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_conflicts_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_roadannotations_node)
target_sources(plot_roadannotations_node PRIVATE src/plot_roadannotations_node.cpp)
target_include_directories(plot_roadannotations_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_roadannotations_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          plotlablib::plotlablib
                          dlib::dlib
)
# target_compile_options(plot_roadannotations_node BEFORE PRIVATE -g1)

add_executable(plot_plan_swath_node)
target_sources(plot_plan_swath_node PRIVATE src/plot_plan_swath_node.cpp)
target_include_directories(plot_plan_swath_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_plan_swath_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::fun 
                          plotlablib::plotlablib
)

add_executable(plot_views_node)
target_sources(plot_views_node PRIVATE src/plot_views_node.cpp)
target_include_directories(plot_views_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plot_views_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::sim 
                          libadore::mad 
                          plotlablib::plotlablib
)


add_executable(adore_pvprovider_node)
target_sources(adore_pvprovider_node PRIVATE src/adore_pvprovider_node.cpp)
target_include_directories( adore_pvprovider_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_pvprovider_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::env 
                          libadore::sim 
                          libadore::apps 
                          libadore::params
)


# target_include_directories(
#  test_trajectory_planner_node
#  PRIVATE
#  ${proj_includes}
# #  ${CMAKE_INSTALL_PREFIX}/include
# )

add_executable(test_lc_trajectory_planner_node src/test_lc_trajectory_planner_node.cpp)
target_include_directories(test_lc_trajectory_planner_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_lc_trajectory_planner_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::sim 
                          libadore::params 
                          libadore::fun
                          plotlablib::plotlablib
)

# target_include_directories(
#  test_lc_trajectory_planner_node
#  PRIVATE
#  ${proj_includes}
# #  ${CMAKE_INSTALL_PREFIX}/include
# )

add_executable(test_setspeedlimit src/test_setspeedlimit.cpp)
target_include_directories(test_setspeedlimit PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_setspeedlimit 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim
)

add_executable(test_checkpoint_clearance src/test_checkpoint_clearance.cpp)
target_include_directories(test_checkpoint_clearance PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_checkpoint_clearance 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim
)


add_executable(adore_lfbehavior_node src/adore_lfbehavior_node.cpp)
target_include_directories(adore_lfbehavior_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_lfbehavior_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          dlib::dlib
)

add_executable(adore_trajectory_planner_lf_node src/adore_trajectory_planner_lf_node.cpp)
target_include_directories(adore_trajectory_planner_lf_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_trajectory_planner_lf_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          libadore::if_r2s 
                          dlib::dlib
)

add_executable(adore_trajectory_planner_lm_node src/adore_trajectory_planner_lm_node.cpp)
target_include_directories(
    adore_trajectory_planner_lm_node 
      PRIVATE 
        include 
        ${catkin_INCLUDE_DIRS}
        ${adore_if_ros_msg_INCLUDE_DIRS}
)
target_link_libraries(adore_trajectory_planner_lm_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          dlib::dlib
)

add_executable(adore_trajectory_planner_lc_node src/adore_trajectory_planner_lc_node.cpp)
target_include_directories(adore_trajectory_planner_lc_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_trajectory_planner_lc_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
)

add_executable(adore_trajectory_planner_alc_node src/adore_trajectory_planner_alc_node.cpp)
target_include_directories(adore_trajectory_planner_alc_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_trajectory_planner_alc_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::env 
                          libadore::sim 
                          dlib::dlib
)

add_executable(adore_tactical_planner_node src/adore_tactical_planner_node.cpp)
target_include_directories(adore_tactical_planner_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_tactical_planner_node 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::fun 
                          libadore::sim 
                          plotlablib::plotlablib 
                          dlib::dlib
)
# target_compile_options(adore_tactical_planner_node BEFORE PRIVATE -g1) 

add_executable(area_of_effect_provider src/area_of_effect_provider.cpp)
target_include_directories(area_of_effect_provider PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(area_of_effect_provider 
                        PRIVATE 
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::env 
                          dlib::dlib
)

add_executable(ego_transform_provider src/ego_transform_provider.cpp)
target_include_directories(ego_transform_provider PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(ego_transform_provider 
                        PRIVATE 
                        ${catkin_LIBRARIES} 
)

#add_executable(adore_coincide_planner_node src/adore_coincide_planner_node.cpp)
#target_include_directories(adore_coincide_planner_node PRIVATE include ${catkin_INCLUDE_DIRS})
#target_link_libraries(adore_coincide_planner_node PRIVATE ${catkin_LIBRARIES} libadore::apps libadore::params libadore::fun libadore::sim)

add_executable(adore_monitor0_node src/adore_monitor0_node.cpp)
target_include_directories(adore_monitor0_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(adore_monitor0_node 
                        PRIVATE 
                        ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::params 
                          libadore::env 
                          libadore::sim
)

add_executable(plainxmlexporter src/plainxmlexporter.cpp)
target_include_directories(plainxmlexporter PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(plainxmlexporter 
                        PUBLIC
                          ${catkin_LIBRARIES} 
                          libadore::apps 
                          libadore::env 
                          libadore::if_xodr 
                          libadore::if_r2s 
                          plotlablib::plotlablib
)


## Specify libraries to link a library or executable target against
# link against target adore_if_xodr instead of library
# target_link_libraries(test_trajectory_planner_node
#    ${catkin_LIBRARIES}
#    ${LAPACK_LIBRARIES}
#    ${qpOASES} 
#    ${LAPACK_LIBRARIES}
#    ${ZeroMQ_LIBRARIES}
# )

# ## Specify libraries to link a library or executable target against
# # link against target adore_if_xodr instead of library
# target_link_libraries(test_lc_trajectory_planner_node
#    ${catkin_LIBRARIES}
#    ${LAPACK_LIBRARIES}
#    ${qpOASES} 
#    ${LAPACK_LIBRARIES}
#    ${ZeroMQ_LIBRARIES}
# )


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

# catkin_install_python(PROGRAMS
#  # python scripts listed here
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY 
# # directory name here
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
# FILES_MATCHING PATTERN "*.py" # pattern example for python scripts
#)
set(CMAKE_VERBOSE_MAKEFILE ON)
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS adore_vehiclemodel_node adore_timer_node adore_feedbackcontroller_node adore_mapprovider_node adore_borderbird_node vehicle_udp_gateway_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(
  TARGETS 
    adore_vehiclemodel_node
    adore_odometrymodel_node
    adore_localizationmodel_node
    adore_objectdetectionmodel_node
    adore_timer_node
    adore_feedbackcontroller_node
    adore_mapprovider_node
    adore_speedlimitprovider_node
    adore_indicatorhintprovider_node
    adore_navigation_node
    plot_graph_node
    test_trajectory_planner_node
    test_mrm_planner_node
    adore_lfbehavior_node
    adore_trajectory_planner_lf_node
    adore_trajectory_planner_lm_node
    adore_trajectory_planner_lc_node
    adore_trajectory_planner_alc_node
    adore_tactical_planner_node
    test_lc_trajectory_planner_node
    test_straight_line_prediction_node
    adore_gap_provider
    adore_prediction_provider
    adore_prediction_filter
    adore_scheduler_node
    adore_monitor0_node
#    adore_coincide_planner_node
    adore_lvprovider_node
    adore_checkpoint_controller_node
    adore_mission_controller_node
    adore_crosstraffic_provider
    area_of_effect_provider
    ego_transform_provider
    plainxmlexporter
    test_control_dashboard
    test_control_error_node
    test_setspeedlimit
    test_checkpoint_clearance
    plot_area_of_effect
    plot_ego_node
    plot_gaps_node
    plot_lanes_node
    plot_planning_details_node
    plot_predictions_node
    plot_predictions_minimal_node
    plot_conflicts_node
    plot_plan_swath_node
    plot_satimages_node
    plot_scalar_node
    plot_vector_node
    plot_traffic_node
    plot_trafficlights_node
    plot_views_node
    plot_trajectories_node
    plot_roadannotations_node
    adore_pvprovider_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  PROGRAMS 
  src/adore_ci_terminator_node.py
  src/adore_set_pose.py
  src/adore_set_goal.py
  src/adore_suppress_lanechanges.py
  src/test_vectorsize.py
  src/plotlab2navigationgoal.py
  src/test_usb_keyboard.py
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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
