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

project(plotlablib VERSION 0.0.1
             DESCRIPTION "Plotting library as part of the CSA framework"
             HOMEPAGE_URL "https://github.com/eclipse/adore/tree/master/plotlab/lib")

include(GNUInstallDirs)

set_directory_properties(PROPERTIES EXCLUDE_FROM_ALL YES)

include(external/zmq.cmake)

add_library(plotlablib INTERFACE)

get_directory_property(PLOTLABLIB_PARENT_DIR PARENT_DIRECTORY)
if(NOT "${PLOTLABLIB_PARENT_DIR}" STREQUAL "")
message(STATUS " No compiler Warnings for Plotlablib since it is a subdir of ${PLOTLABLIB_PARENT_DIR}")

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            ${zmqlib_include})

target_link_libraries(plotlablib INTERFACE zmq)

else()

message(STATUS " plotlablib building as seperate test")

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            ${zmqlib_include})
target_compile_options(${PROJECT_NAME} INTERFACE "-Wall")

endif()



target_link_libraries(plotlablib INTERFACE zmq)

target_compile_features(plotlablib INTERFACE cxx_std_17)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE adore::
    DESTINATION ../cmake/${PROJECT_NAME}
)

export(EXPORT ${PROJECT_NAME}Targets
       FILE "${CMAKE_BINARY_DIR}/../cmake/${PROJECT_NAME}Targets.cmake"
       NAMESPACE adore::
)


# add executable to test compilation of the header only library
# but it is build only if this is the toplevel cmake project
if("${PLOTLABLIB_PARENT_DIR}" STREQUAL "")
add_executable(plotlab_test_compile test/main.cpp)
target_link_libraries(plotlab_test_compile PRIVATE plotlablib)
endif()
