# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# * Thomas Lobig
# ********************************************************************************


include (FetchContent)
FetchContent_Declare(zmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
    GIT_TAG        v4.3.2
)
FetchContent_GetProperties(zmq)
if(NOT zmq_POPULATED)
    FetchContent_Populate(zmq)

    # disable unwanted parts of the building process
    unset(zmq_is_cmakelists_fixed)
    unset(zmq_temp_cmakelists)
    file(READ ${zmq_SOURCE_DIR}/CMakeLists.txt zmq_temp_cmakelists)
    string(FIND "${zmq_temp_cmakelists}" "#disabled_unwanted_" zmq_is_cmakelists_fixed)

    if(${zmq_is_cmakelists_fixed} EQUAL -1)
    # this silences a cmake dev error by handling policy CMP0077
    string(REPLACE "include(ZMQSupportMacros)" "include(ZMQSupportMacros) ##disabled_unwanted_errors\ncmake_policy(SET CMP0077 NEW)" zmq_temp_cmakelists "${zmq_temp_cmakelists}")
    file(WRITE ${zmq_SOURCE_DIR}/CMakeLists.txt "${zmq_temp_cmakelists}")
    endif()
    cmake_policy(SET CMP0077 NEW)
    set(BUILD_TESTS OFF)
    set(WITH_DOCS OFF)
    set(BUILD_SHARED OFF)
    set(ENABLE_CPACK OFF)

    cmake_policy(SET CMP0048 NEW)
    add_subdirectory(${zmq_SOURCE_DIR} ${zmq_BINARY_DIR})

endif()

