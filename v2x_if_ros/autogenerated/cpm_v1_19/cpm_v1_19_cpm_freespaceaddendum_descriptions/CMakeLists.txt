# 
# Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
# Institute of Transportation Systems. (https://www.dlr.de/ts/)
# 
# Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# 
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
# 
# 
# File automatically generated with DLR Wind v2 (2021)
# 
# This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3
# 
# Module: CPM_FreeSpaceAddendum_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
#     wg1(1) ts(103324) freeSpaceAddendum(6) version1(1)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(cpm_v1_19_cpm_freespaceaddendum_descriptions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    cpm_v1_19_cpm_commondatatypes_descriptions
)
find_package(Boost)

add_message_files(
    FILES
    FreeSpaceAddendum.msg
    FreeSpaceAddendumContainer.msg
    FreeSpaceArea.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    cpm_v1_19_cpm_commondatatypes_descriptions
)

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs cpm_v1_19_cpm_commondatatypes_descriptions
    # CATKIN_DEPENDS message_runtime roscpp
)

add_library(cpm_v1_19_cpm_freespaceaddendum_descriptions INTERFACE)

add_dependencies(cpm_v1_19_cpm_freespaceaddendum_descriptions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS cpm_v1_19_cpm_freespaceaddendum_descriptions FILE cpm_v1_19_cpm_freespaceaddendum_descriptions.cmake)

