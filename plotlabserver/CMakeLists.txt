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
#*   Thomas Lobig - initial API and implementation
#********************************************************************************
cmake_minimum_required(VERSION 3.13)

project(plotlabserver VERSION 0.0.1
             DESCRIPTION "Plotting server as part of the CSA framework"
             HOMEPAGE_URL "https://gitlab.dlr.de/csa/plotlabserver"
)

set(CMAKE_MODULE_PATH
#    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake
#    ${CMAKE_CURRENT_SOURCE_DIR}/../external
    CACHE INTERNAL "" FORCE
)
          
include(GNUInstallDirs)


set(plotlablibdir ${CMAKE_CURRENT_SOURCE_DIR}/plotlablib)
if(NOT EXISTS ${plotlablibdir})
message(NOTICE "plotlablib sources must be downloaded")
  find_package(Git REQUIRED)
  execute_process(
      COMMAND ${GIT_EXECUTABLE} clone https://gitlab.dlr.de/csa/plotlablib.git ${plotlablibdir}
  )
endif()
set(stbimagepath ${CMAKE_CURRENT_SOURCE_DIR}/include/plotlabserver/)
if(NOT EXISTS ${stbimagepath}stb_image.h)
message(NOTICE "stb_image header must be downloaded")
  execute_process(
      COMMAND wget https://raw.githubusercontent.com/nothings/stb/master/stb_image.h --directory-prefix=${stbimagepath}
  )
endif()

add_subdirectory("${plotlablibdir}" "${CMAKE_CURRENT_BINARY_DIR}/plotlablib")

add_executable(plotlabserver)

find_package(Threads REQUIRED)

find_package(GLUT REQUIRED)

find_package(OpenGL REQUIRED)

FIND_PACKAGE(CURL)

target_sources(plotlabserver
  PRIVATE
    src/circleplot.cpp
    src/figure.cpp
    src/lineplot.cpp
    src/patchplot.cpp
    src/plotlabserver.cpp
    src/plotobject.cpp
    src/stb_image.cpp
    src/textplot.cpp
    src/texturecache.cpp
    src/textureplot.cpp
    src/tristrip.cpp
)

target_include_directories(plotlabserver PRIVATE include)

target_link_libraries(plotlabserver PRIVATE plotlablib Threads::Threads ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} CURL::libcurl)

target_compile_features(plotlabserver PRIVATE cxx_std_17)
# target_compile_options(plotlabserver POSITION_INDEPENDENT_CODE ON)
set_target_properties(plotlabserver PROPERTIES POSITION_INDEPENDENT_CODE ON THREADS_PREFER_PTHREAD_FLAG ON)