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
# Module: AddGrpC {iso(1) standard(0) signalizedIntersection(19091) profilec(2) 
#     addgrpc(0) version2(2)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(dsrc_v2_addgrpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    dsrc_v2_dsrc
    its_container_v2_its_container
)
find_package(Boost)

add_message_files(
    FILES
    BatteryStatus.msg
    ConnectionManeuverAssist_addGrpC.msg
    ConnectionTrajectory_addGrpC.msg
    EmissionType.msg
    ExceptionalCondition.msg
    IntersectionState_addGrpC.msg
    ItsStationPosition.msg
    ItsStationPositionList.msg
    LaneAttributes_addGrpC.msg
    MapData_addGrpC.msg
    MovementEvent_addGrpC.msg
    Node.msg
    NodeAttributeSet_addGrpC.msg
    NodeLink.msg
    Node_id.msg
    Position3D_addGrpC.msg
    PrioritizationResponse.msg
    PrioritizationResponseList.msg
    PtvRequestType.msg
    RejectedReason.msg
    RequestorDescription_addGrpC.msg
    RestrictionUserType_addGrpC.msg
    SignalHeadLocation.msg
    SignalHeadLocationList.msg
    SignalStatusPackage_addGrpC.msg
    TimeReference.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    its_container_v2_its_container
    dsrc_v2_dsrc
)

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs its_container_v2_its_container dsrc_v2_dsrc
    # CATKIN_DEPENDS message_runtime roscpp
)

add_library(dsrc_v2_addgrpc INTERFACE)

add_dependencies(dsrc_v2_addgrpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS dsrc_v2_addgrpc FILE dsrc_v2_addgrpc.cmake)

