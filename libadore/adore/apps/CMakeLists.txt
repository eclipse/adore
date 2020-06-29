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

add_library(${PROJECT_NAME} STATIC)

include(../../external/plotlablib.cmake)

target_sources(adore_apps
  PRIVATE
    src/apps.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(adore_apps PUBLIC adore_params adore_env plotlablib)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
