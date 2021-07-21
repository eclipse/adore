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
#*   Thomas Lobig
#********************************************************************************


cmake_minimum_required(VERSION 3.13)

project("adore_env" VERSION 0.0.1
             DESCRIPTION "Environment representation classes as part of the adore framework"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/env")

include(GNUInstallDirs)

add_library(${PROJECT_NAME} INTERFACE)

#  TODO this is currenty only needed in lanefollowing geometry, but lands in all targets
#  since this is undesireable, maybe more specific build targets are needed
include(../../external/csaps-cpp.cmake)

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(adore_env INTERFACE adore_mad adore_view adore_fun csaps)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# target_compile_options(${PROJECT_NAME}  BEFORE INTERFACE -pg)
# target_link_options(${PROJECT_NAME}  BEFORE INTERFACE -pg)

if(BUILD_adore_TESTING) 
add_subdirectory(test)
endif()

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