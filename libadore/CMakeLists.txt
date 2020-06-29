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

project("adore_libs" VERSION 0.0.1.0
             DESCRIPTION "Automated Driving Open Research: Toolkit libraries"
             HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore"
)

set_directory_properties(PROPERTIES EXCLUDE_FROM_ALL YES)

include(GNUInstallDirs)
include(CTest)

#option() does nothing if option already defined
option(BUILD_adore_TESTING OFF)

if(BUILD_adore_TESTING)
  message(NOTICE "Option BUILD_adore_TESTING is set, will build adore tests.")
  option(CATCH_BUILD_TESTING ON)
  add_subdirectory(external/catch2)
  set (CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/catch2/catch2_external_sources/contrib/
  )
endif()

add_subdirectory(adore "${CMAKE_CURRENT_BINARY_DIR}/adore")
