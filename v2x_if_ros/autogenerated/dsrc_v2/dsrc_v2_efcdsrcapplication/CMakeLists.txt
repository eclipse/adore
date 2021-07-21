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
# Module: EfcDsrcApplication {iso(1) standard(0) application(0) version5(5)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(dsrc_v2_efcdsrcapplication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    dsrc_v2_aviaeinumberinganddatastructures
    dsrc_v2_efcdsrcgeneric
)
find_package(Boost)

add_message_files(
    FILES
    AxleWeightLimits.msg
    CopValue.msg
    DieselEmissionValues.msg
    DieselEmissionValues_particulate.msg
    DieselEmissionValues_particulate_value.msg
    EngineCharacteristics.msg
    EnvironmentalCharacteristics.msg
    EuroValue.msg
    ExhaustEmissionValues.msg
    ExhaustEmissionValues_emissionCO.msg
    Int1.msg
    Int2.msg
    PassengerCapacity.msg
    Provider.msg
    SoundLevel.msg
    UnitType.msg
    VehicleDimensions.msg
    VehicleWeightLimits.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    dsrc_v2_efcdsrcgeneric
    dsrc_v2_aviaeinumberinganddatastructures
)

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs dsrc_v2_efcdsrcgeneric dsrc_v2_aviaeinumberinganddatastructures
    # CATKIN_DEPENDS message_runtime roscpp
)

add_library(dsrc_v2_efcdsrcapplication INTERFACE)

add_dependencies(dsrc_v2_efcdsrcapplication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS dsrc_v2_efcdsrcapplication FILE dsrc_v2_efcdsrcapplication.cmake)

