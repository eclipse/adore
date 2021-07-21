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
FetchContent_Declare(csaps
    GIT_REPOSITORY https://github.com/tlobig/csaps-cpp.git #TODO put that branch in DLR-TS and fetch from there
    # GIT_TAG        tfn2
)
FetchContent_GetProperties(csaps)
if(NOT csaps_POPULATED)
    FetchContent_Populate(csaps)
    add_subdirectory(${csaps_SOURCE_DIR} ${csaps_BINARY_DIR})
    set_property(DIRECTORY ${csaps_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)


    install(TARGETS csaps EXPORT csapsTargets
    )

    install(EXPORT csapsTargets
        FILE csapsTargets.cmake
        NAMESPACE adore::
        DESTINATION ../cmake/csaps
    )

    export(EXPORT csapsTargets
        FILE "${CMAKE_BINARY_DIR}/../cmake/csapsTargets.cmake"
        NAMESPACE adore::
    )

endif()