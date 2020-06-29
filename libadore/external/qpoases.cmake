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

include (FetchContent)
set(FETCHCONTENT_QUIET off)
FetchContent_Declare(qpoases
URL https://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.1.zip
) # URL hash?
FetchContent_GetProperties(qpoases)
if(NOT qpoases_POPULATED)
    FetchContent_Populate(qpoases)
    option(QPOASES_BUILD_EXAMPLES "Build examples." OFF)

    # disable unwanted parts of the building process
    unset(qpoases_is_cmakelists_fixed)
    unset(qpoases_temp_cmakelists)
    file(READ ${qpoases_SOURCE_DIR}/CMakeLists.txt qpoases_temp_cmakelists)
    string(FIND "${qpoases_temp_cmakelists}" "#disabled_unwanted_" qpoases_is_cmakelists_fixed)

    if(${qpoases_is_cmakelists_fixed} EQUAL -1)
    # this silences a cmake dev error by handling policy CMP0048
    string(REPLACE "PROJECT(qpOASES CXX)" "cmake_policy(SET CMP0048 NEW) \n  PROJECT(qpOASES LANGUAGES CXX VERSION 3.2.0) ##disabled_unwanted_dd_subdirectory(doc)" qpoases_temp_cmakelists "${qpoases_temp_cmakelists}")
    file(WRITE ${qpoases_SOURCE_DIR}/CMakeLists.txt "${qpoases_temp_cmakelists}")
    endif()

    PROJECT(qpOASES CXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    add_subdirectory(${qpoases_SOURCE_DIR} ${qpoases_BINARY_DIR})
endif()


