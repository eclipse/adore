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
# This file belongs to the DLR Wind project de.dlr.ts.v2x:dsrc_v2:2.5
# 
# Module: ElectronicRegistrationIdentificationVehicleDataModule {iso(1) standard(0) iso24534(24534) vehicleData(1) version1(1)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(dsrc_v2_electronicregistrationidentificationvehicledatamodule)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)
find_package(Boost)

add_message_files(
    FILES
    EuVehicleCategoryCode.msg
    EuVehicleCategoryL.msg
    EuVehicleCategoryM.msg
    EuVehicleCategoryN.msg
    EuVehicleCategoryO.msg
    Iso3833VehicleType.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs
    # CATKIN_DEPENDS message_runtime roscpp
)

add_library(dsrc_v2_electronicregistrationidentificationvehicledatamodule INTERFACE)

add_dependencies(dsrc_v2_electronicregistrationidentificationvehicledatamodule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS dsrc_v2_electronicregistrationidentificationvehicledatamodule FILE dsrc_v2_electronicregistrationidentificationvehicledatamodule.cmake)

