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
#FetchContent_Declare(xodr
#    GIT_REPOSITORY https://github.com/DLR-TS/xodr.git
#    GIT_TAG        avoid_name_clashes
#)
#the same with testfeld niedersachsen xsd:
FetchContent_Declare(xodr
    GIT_REPOSITORY https://github.com/daniel-gitgit/xodr.git
    GIT_TAG        tfn2
)
FetchContent_GetProperties(xodr)
if(NOT xodr_POPULATED)
    FetchContent_Populate(xodr)
    add_subdirectory(${xodr_SOURCE_DIR} ${xodr_BINARY_DIR})
endif()

