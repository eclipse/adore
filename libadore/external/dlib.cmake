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
FetchContent_Declare(dlib
    GIT_REPOSITORY https://github.com/davisking/dlib.git
    GIT_TAG        v19.19
)
FetchContent_GetProperties(dlib)
if(NOT dlib_POPULATED)
    FetchContent_Populate(dlib)

    # disable unwanted parts of the building process
    # option(DLIB_ENABLE_ASSERTS ${DLIB_ENABLE_ASSERTS_STR} OFF)
    # option(DLIB_ISO_CPP_ONLY ${DLIB_ISO_CPP_ONLY_STR} OFF)
    # # toggle_preprocessor_switch(DLIB_ISO_CPP_ONLY)
    # option(DLIB_NO_GUI_SUPPORT ${DLIB_NO_GUI_SUPPORT_STR} OFF)
    # # toggle_preprocessor_switch(DLIB_NO_GUI_SUPPORT)
    # option(DLIB_ENABLE_STACK_TRACE ${DLIB_ENABLE_STACK_TRACE_STR} OFF)
    # # toggle_preprocessor_switch(DLIB_ENABLE_STACK_TRACE)
    # option(DLIB_USE_MKL_SEQUENTIAL ${DLIB_USE_MKL_SEQUENTIAL_STR} OFF)
    # option(DLIB_USE_MKL_WITH_TBB ${DLIB_USE_MKL_WITH_TBB_STR} OFF)

    # option(DLIB_JPEG_SUPPORT ${DLIB_JPEG_SUPPORT_STR} OFF)
    # option(DLIB_LINK_WITH_SQLITE3 ${DLIB_LINK_WITH_SQLITE3_STR} OFF)
    # option(DLIB_USE_BLAS ${DLIB_USE_BLAS_STR} OFF)
    # option(DLIB_USE_LAPACK ${DLIB_USE_LAPACK_STR} OFF)
    # option(DLIB_USE_CUDA ${DLIB_USE_CUDA_STR} OFF)
    # option(DLIB_PNG_SUPPORT ${DLIB_PNG_SUPPORT_STR} OFF)
    # option(DLIB_GIF_SUPPORT ${DLIB_GIF_SUPPORT_STR} OFF)
    # #option(DLIB_USE_FFTW ${DLIB_USE_FFTW_STR} OFF)
    # option(DLIB_USE_MKL_FFT ${DLIB_USE_MKL_FFT_STR} OFF)

    set(temp_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # disable warning for software we have no control over
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    cmake_policy(SET CMP0048 NEW)
    add_subdirectory(${dlib_SOURCE_DIR} ${dlib_BINARY_DIR})
    set(CMAKE_CXX_FLAGS "${temp_CMAKE_CXX_FLAGS}")
endif()

