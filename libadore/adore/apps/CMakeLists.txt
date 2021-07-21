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
#*   Thomas Lobig - initial implementation
#********************************************************************************

cmake_minimum_required(VERSION 3.13)

project("adore_apps" VERSION 0.0.1
             DESCRIPTION "abstract apps as part of the adore framework"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/apps")

include(GNUInstallDirs)

add_library(${PROJECT_NAME} INTERFACE)

if(NOT TARGET plotlablib)
  add_subdirectory(../../../plotlab/lib ${CMAKE_BINARY_DIR}/../plotlablib)
endif()


target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(
  ${PROJECT_NAME} SYSTEM INTERFACE ${plotlablib_include_dirs}
)

# TODO target apps pulls in all dependencies for all apps, should be seperated out
target_link_libraries(adore_apps INTERFACE adore_params adore_env adore_if_r2s plotlablib adore_sim)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

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