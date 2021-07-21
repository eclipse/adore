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

set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/../_deps)

FetchContent_Declare(qpoases
    GIT_REPOSITORY https://github.com/coin-or/qpOASES/
    GIT_TAG        releases/3.2.1
)
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

    # disable further unwanted parts of the building process
    unset(qpoases_is_cmakelists_part2_fixed)
    unset(qpoases_temp_part2_cmakelists)
    file(READ ${qpoases_SOURCE_DIR}/CMakeLists.txt qpoases_temp_part2_cmakelists)
    string(FIND "${qpoases_temp_part2_cmakelists}" "#disabled_further_unwanted_" qpoases_is_cmakelists_part2_fixed)

    if(${qpoases_is_cmakelists_part2_fixed} EQUAL -1)
    # this silences a cmake dev error by handling policy CMP0048
    string(REPLACE "-Wall -pedantic -Wfloat-equal -Wshadow -DLINUX\")" "-w -DLINUX\") ##disabled_further_unwanted_warnings" qpoases_temp_part2_cmakelists "${qpoases_temp_part2_cmakelists}")
    file(WRITE ${qpoases_SOURCE_DIR}/CMakeLists.txt "${qpoases_temp_part2_cmakelists}")
    endif()

    PROJECT(qpOASES CXX)
    add_subdirectory(${qpoases_SOURCE_DIR} ${qpoases_BINARY_DIR})
    set_property(DIRECTORY ${qpoases_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)

    install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
)

    install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE adore::
        DESTINATION ../cmake/${PROJECT_NAME}
    )

    export(EXPORT ${PROJECT_NAME}Targets
        FILE "${CMAKE_BINARY_DIR}/../cmake/${PROJECT_NAME}Targets.cmake"
        NAMESPACE adore::
    )
endif()


