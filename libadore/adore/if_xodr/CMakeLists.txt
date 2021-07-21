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

project(adore_if_xodr VERSION 0.0.1
             DESCRIPTION "Open drive interface as part of the adore framework"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/if_xodr")

include(GNUInstallDirs)

add_library(adore_if_xodr STATIC)

target_sources(adore_if_xodr
  PRIVATE
    src/xodr2borderbased.cpp 
)

target_include_directories(
  adore_if_xodr
  PUBLIC $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

include(../../external/xodr.cmake)

target_link_libraries(adore_if_xodr
  PUBLIC    
    adore::mad
    adore::env
    xodr::xodr
)
target_compile_features(adore_if_xodr PUBLIC cxx_std_17)

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