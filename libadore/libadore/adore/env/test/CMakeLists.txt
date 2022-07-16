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
#*   Daniel Heß
#********************************************************************************

cmake_minimum_required(VERSION 3.13)
include(GNUInstallDirs)
if(BUILD_adore_TESTING) #why not always build tests?
# adding test cases for the library
  # add_subdirectory(../../../../../plotlab/lib ${CMAKE_BINARY_DIR}/../plotlablib)
  find_package(Catch2 CONFIG)
  # find_package(plotlablib CONFIG REQUIRED)
  # cases correspond to .cpp files in folder test
  add_executable(test_adore_env) 
  target_sources(test_adore_env PRIVATE 
    test_adore_env.cpp
    test_controlledconnection.cpp
    test_occupancycylinder.cpp
    test_occupancycylinderprediction.cpp
    test_ocroadbasedprediction.cpp
    test_lanechangeborders.cpp
    test_independentlanechangegeometry.cpp
    test_baseline.cpp
  )
  target_link_libraries(test_adore_env PRIVATE env Catch2::Catch2 if_xodr apps)
  target_compile_features(test_adore_env PRIVATE cxx_std_17)

  include(CTest)
  include(Catch)
  catch_discover_tests(test_adore_env)

endif()
