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

project(adore_view VERSION 0.0.1
             DESCRIPTION "view interface classes, part of the adore framework"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/view")

include(GNUInstallDirs)

add_library(adore_view INTERFACE)

target_include_directories(
  adore_view
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(adore_view INTERFACE cxx_std_17)
