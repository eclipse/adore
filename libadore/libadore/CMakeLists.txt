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

project(
    "libadore" VERSION 0.0.1.0
    DESCRIPTION "Automated Driving Open Research: Toolkit libraries"
    HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore"
)

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "Automated Driving Open Research: Toolkit libraries")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)

set_directory_properties(PROPERTIES EXCLUDE_FROM_ALL NO)

include(GNUInstallDirs)
include(CTest)

set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/../_deps)

#option() does nothing if option already defined
option(BUILD_adore_TESTING ON)

if(BUILD_adore_TESTING)
  message(STATUS "Option BUILD_adore_TESTING is set, will build adore tests.")
  option(CATCH_BUILD_TESTING ON)
  add_subdirectory(external/Catch2)
  set (CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Catch2/contrib/
  )
endif()

#option() does nothing if option already defined
option(BUILD_adore_DEBUG_BUILD_PROCESS OFF)

if(BUILD_adore_DEBUG_BUILD_PROCESS)
  add_compile_options(-save-temps=obj)
endif()
add_subdirectory(adore "${CMAKE_CURRENT_BINARY_DIR}/adore")






