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

project(adore_mad VERSION 0.0.1
             DESCRIPTION "mathematical and algorithmic classes and functions as part of the adore framework"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/mad")

include(GNUInstallDirs)

add_library(adore_mad STATIC)

find_package(Boost REQUIRED)

include(../../external/qpoases.cmake)
include(../../external/dlib.cmake)


target_sources(adore_mad 
  PRIVATE
 #src/array_matrix_tools.cpp
  src/cubic_piecewise_function.cpp
  src/csvlog.cpp
)

target_include_directories(
  adore_mad
  PUBLIC 
    $<BUILD_INTERFACE:${adore_mad_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(
  adore_mad
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${qpoases_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(adore_mad PUBLIC adore_params dlib::dlib Boost::headers qpOASES)

target_compile_features(adore_mad PUBLIC cxx_std_17)

target_compile_options(adore_mad PRIVATE
                           $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
                                -Wall>
                           $<$<CXX_COMPILER_ID:MSVC>:
                                W4>)
if(BUILD_adore_TESTING) 
add_subdirectory(test)
endif()

install(TARGETS adore_mad EXPORT adore_madTargets
)

install(EXPORT adore_madTargets
    FILE adore_madTargets.cmake
    NAMESPACE adore::
    DESTINATION ../cmake/adore_mad
)

export(EXPORT adore_madTargets
       FILE "${CMAKE_BINARY_DIR}/../cmake/adore_madTargets.cmake"
       NAMESPACE adore::
)