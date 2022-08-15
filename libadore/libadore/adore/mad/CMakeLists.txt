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

project(
    mad VERSION 0.0.1
    DESCRIPTION "mathematical and algorithmic classes and functions as part of the adore framework"
    HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/mad"
)

include(GNUInstallDirs)

add_library(${PROJECT_NAME} STATIC)


add_library(dlib::dlib STATIC IMPORTED GLOBAL)
set_property(TARGET dlib::dlib PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/../../external/dlib/build/install/lib/libdlib.a")
target_include_directories(dlib::dlib INTERFACE "${CMAKE_BINARY_DIR}/install/include")

add_library(qpoases::qpoases STATIC IMPORTED GLOBAL)
set_property(TARGET qpoases::qpoases PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../../external/qpOASES/build/install/lib/libqpOASES.a)

find_package(Boost REQUIRED)

target_link_libraries(
  ${PROJECT_NAME} 
    PUBLIC
      dlib::dlib
      qpoases::qpoases
      blas
      lapack
)

target_sources(${PROJECT_NAME}
  PRIVATE
 #src/array_matrix_tools.cpp
  src/cubic_piecewise_function.cpp
  src/csvlog.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${qpoases_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_options(${PROJECT_NAME} PRIVATE
                           $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
                                -Wall>
                           $<$<CXX_COMPILER_ID:MSVC>:
                                W4>)
if(BUILD_adore_TESTING) 
add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)


install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE libadore::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              # "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION .)

