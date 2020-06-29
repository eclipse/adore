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
FetchContent_Declare(plotlablib
    GIT_REPOSITORY https://gitlab.dlr.de/csa/plotlablib.git
) # add GIT_TAG, if needed

FetchContent_GetProperties(plotlablib)
if(NOT plotlablib_POPULATED)
    FetchContent_Populate(plotlablib)
    # could add further cmake configuration items here
    add_subdirectory(${plotlablib_SOURCE_DIR} ${plotlablib_BINARY_DIR})
endif()

