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


include (FetchContent)
set(FETCHCONTENT_QUIET off)
FetchContent_Declare(xercesc
GIT_REPOSITORY https://github.com/apache/xerces-c.git
GIT_TAG        v3.2.2
)
FetchContent_GetProperties(xercesc)
if(NOT xercesc_POPULATED)
    FetchContent_Populate(xercesc)

    # disable unwanted parts of the building process
    unset(xerces_is_cmakelists_fixed)
    unset(xercesc_temp_cmakelists)
    file(READ ${xercesc_SOURCE_DIR}/CMakeLists.txt xercesc_temp_cmakelists)
    string(FIND "${xercesc_temp_cmakelists}" "#disabled_unwanted_" xerces_is_cmakelists_fixed)

    if(${xerces_is_cmakelists_fixed} EQUAL -1)
    string(REPLACE "add_subdirectory(doc)" "##disabled_unwanted_dd_subdirectory(doc)" xercesc_temp_cmakelists "${xercesc_temp_cmakelists}")
    string(REPLACE "add_subdirectory(tests)" "#_dd_subdirectory(tests)" xercesc_temp_cmakelists "${xercesc_temp_cmakelists}")
    string(REPLACE "add_subdirectory(samples)" "#_dd_subdirectory(samples)" xercesc_temp_cmakelists "${xercesc_temp_cmakelists}")
    file(WRITE ${xercesc_SOURCE_DIR}/CMakeLists.txt "${xercesc_temp_cmakelists}")
    endif()
    set(temp_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # disable warning for software we have no control over
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") 
    add_subdirectory(${xercesc_SOURCE_DIR} ${xercesc_BINARY_DIR})
    set(CMAKE_CXX_FLAGS "${temp_CMAKE_CXX_FLAGS}")
endif()

