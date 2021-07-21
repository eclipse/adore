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
# Module: CPM_CommonDataTypes_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
#     wg1(1) ts(103324) commonDataTypes(2) version1(1)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(cpm_v1_19_cpm_commondatatypes_descriptions)

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
    AngleConfidence.msg
    AnimalSubclassType.msg
    AreaCircular.msg
    AreaEllipse.msg
    AreaPolygon.msg
    AreaRadial.msg
    AreaRectangle.msg
    CartesianAngle.msg
    CartesianAngleValue.msg
    ClassConfidence.msg
    DistanceConfidence.msg
    DistanceValue.msg
    DynamicStatus.msg
    FreeSpaceConfidence.msg
    FrontOverhang.msg
    HitchPointOffset.msg
    Identifier.msg
    LongitudinalLanePosition.msg
    LongitudinalLanePositionConfidence.msg
    LongitudinalLanePositionValue.msg
    MatchedPosition.msg
    MessageSegmentInfo.msg
    NodeOffsetPointZ.msg
    NumberOfPerceivedObjects.msg
    ObjectAge.msg
    ObjectClass.msg
    ObjectClassDescription.msg
    ObjectClassWithConfidence.msg
    ObjectConfidence.msg
    ObjectDimension.msg
    ObjectDimensionConfidence.msg
    ObjectDimensionValue.msg
    ObjectDistance.msg
    ObjectRefPoint.msg
    OffsetPoint.msg
    OtherSubclassType.msg
    PersonSubclassType.msg
    PolyPointList.msg
    Radius.msg
    Range.msg
    RearOverhang.msg
    RefPointId.msg
    SegmentCount.msg
    SemiRangeLength.msg
    SensorHeight.msg
    SensorIdList.msg
    SensorType.msg
    ShadowingApplies.msg
    SpeedExtended.msg
    SpeedValueExtended.msg
    TimeOfMeasurement.msg
    TrailerData.msg
    TrailerDataContainer.msg
    UnknownSubclassType.msg
    VehicleHeight.msg
    VehicleSubclassType.msg
    WGS84Angle.msg
    WGS84AngleValue.msg
    XSensorOffset.msg
    YSensorOffset.msg
    ZSensorOffset.msg
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

add_library(cpm_v1_19_cpm_commondatatypes_descriptions INTERFACE)

add_dependencies(cpm_v1_19_cpm_commondatatypes_descriptions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS cpm_v1_19_cpm_commondatatypes_descriptions FILE cpm_v1_19_cpm_commondatatypes_descriptions.cmake)

