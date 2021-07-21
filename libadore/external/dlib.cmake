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

find_package(dlib)

if(NOT dlib_FOUND)

    include (FetchContent)
    set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/../_deps)
    FetchContent_Declare(dlib
        GIT_REPOSITORY https://github.com/davisking/dlib.git
        GIT_TAG        v19.19
    )
    FetchContent_GetProperties(dlib)
    if(NOT dlib_POPULATED)
        FetchContent_Populate(dlib)

        set(temp_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        # disable warning for software we have no control over
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
        cmake_policy(SET CMP0048 NEW)
        add_subdirectory(${dlib_SOURCE_DIR} ${dlib_BINARY_DIR})
        set_property(DIRECTORY ${dlib_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)
        set(CMAKE_CXX_FLAGS "${temp_CMAKE_CXX_FLAGS}")
    endif()

endif() # NOT dlib_FOUND