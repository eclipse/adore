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
             HOMEPAGE_URL "https://github.com/eclipse/adore/plotlabserver/plotlablib")

include(GNUInstallDirs)

include(external/zmq.cmake)

add_library(plotlablib INTERFACE)


target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            ${zmqlib_include})

target_link_libraries(plotlablib INTERFACE zmq)

target_compile_features(plotlablib INTERFACE cxx_std_17)
