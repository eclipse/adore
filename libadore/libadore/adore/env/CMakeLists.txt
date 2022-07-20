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
    "env" 
    VERSION 0.0.1
    DESCRIPTION "Environment representation classes as part of the adore framework"
    HOMEPAGE_URL "${adore_REPOSITORY_URL}/libadore/adore/env"
)

include(GNUInstallDirs)

add_library(${PROJECT_NAME} STATIC)

#list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/Catch2/build/install")
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/csaps-cpp/build/install")


foreach(path ${CMAKE_PREFIX_PATH})
  message("Path: " ${path})
endforeach(path)

find_package(csaps-cpp CONFIG REQUIRED)


target_sources(${PROJECT_NAME}
  PRIVATE
    src/adore_env.cpp 
)

include(GNUInstallDirs)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  mad 
  view 
  fun 
  csaps-cpp::csaps-cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

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

