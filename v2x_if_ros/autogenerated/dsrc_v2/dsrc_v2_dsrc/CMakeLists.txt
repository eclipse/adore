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
# Module: DSRC {iso(1) standard(0) signalizedIntersection(19091) profilec(2) 
#     dsrc(2) version2(2)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(dsrc_v2_dsrc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    dsrc_v2_electronicregistrationidentificationvehicledatamodule
    its_container_v2_its_container
)
find_package(Boost)

add_message_files(
    FILES
    AdvisorySpeed.msg
    AdvisorySpeedList.msg
    AdvisorySpeedType.msg
    AllowedManeuvers.msg
    Angle.msg
    AntennaOffsetSet.msg
    ApproachID.msg
    BasicVehicleRole.msg
    ComputedLane.msg
    ComputedLane_offsetXaxis.msg
    ComputedLane_offsetYaxis.msg
    ConnectingLane.msg
    Connection.msg
    ConnectionManeuverAssist.msg
    ConnectsToList.msg
    DDateTime.msg
    DDay.msg
    DHour.msg
    DMinute.msg
    DMonth.msg
    DOffset.msg
    DSRCmsgID.msg
    DSecond.msg
    DYear.msg
    DataParameters.msg
    DataParameters_geoidUsed.msg
    DataParameters_lastCheckedDate.msg
    DataParameters_processAgency.msg
    DataParameters_processMethod.msg
    DeltaAngle.msg
    DeltaTime.msg
    DescriptiveName.msg
    DrivenLineOffsetLg.msg
    DrivenLineOffsetSm.msg
    Elevation.msg
    ElevationConfidence.msg
    EnabledLaneList.msg
    FuelType.msg
    FullPositionVector.msg
    GNSSstatus.msg
    GenericLane.msg
    HeadingConfidenceDSRC.msg
    HeadingDSRC.msg
    IntersectionAccessPoint.msg
    IntersectionGeometry.msg
    IntersectionGeometryList.msg
    IntersectionID.msg
    IntersectionReferenceID.msg
    IntersectionState.msg
    IntersectionStateList.msg
    IntersectionStatusObject.msg
    LaneAttributes.msg
    LaneAttributes_Barrier.msg
    LaneAttributes_Bike.msg
    LaneAttributes_Crosswalk.msg
    LaneAttributes_Parking.msg
    LaneAttributes_Sidewalk.msg
    LaneAttributes_Striping.msg
    LaneAttributes_TrackedVehicle.msg
    LaneAttributes_Vehicle.msg
    LaneConnectionID.msg
    LaneDataAttribute.msg
    LaneDataAttributeList.msg
    LaneDirection.msg
    LaneID.msg
    LaneList.msg
    LaneSharing.msg
    LaneTypeAttributes.msg
    LaneWidth.msg
    LayerID.msg
    LayerType.msg
    ManeuverAssistList.msg
    MapData.msg
    MergeDivergeNodeAngle.msg
    MinuteOfTheYear.msg
    MovementEvent.msg
    MovementEventList.msg
    MovementList.msg
    MovementPhaseState.msg
    MovementState.msg
    MsgCount.msg
    NodeAttributeSetXY.msg
    NodeAttributeXY.msg
    NodeAttributeXYList.msg
    NodeListXY.msg
    NodeOffsetPointXY.msg
    NodeSetXY.msg
    NodeXY.msg
    Node_LLmD_64b.msg
    Node_XY_20b.msg
    Node_XY_22b.msg
    Node_XY_24b.msg
    Node_XY_26b.msg
    Node_XY_28b.msg
    Node_XY_32b.msg
    Offset_B09.msg
    Offset_B10.msg
    Offset_B11.msg
    Offset_B12.msg
    Offset_B13.msg
    Offset_B14.msg
    Offset_B16.msg
    OverlayLaneList.msg
    PedestrianBicycleDetect.msg
    Position3D.msg
    PositionConfidence.msg
    PositionConfidenceSet.msg
    PositionalAccuracy.msg
    PrioritizationResponseStatus.msg
    PriorityRequestType.msg
    RTCM_Revision.msg
    RTCMcorrections.msg
    RTCMheader.msg
    RTCMmessage.msg
    RTCMmessageList.msg
    RegionId.msg
    RegulatorySpeedLimit.msg
    RequestID.msg
    RequestImportanceLevel.msg
    RequestSubRole.msg
    RequestorDescription.msg
    RequestorPositionVector.msg
    RequestorType.msg
    RestrictionAppliesTo.msg
    RestrictionClassAssignment.msg
    RestrictionClassID.msg
    RestrictionClassList.msg
    RestrictionUserType.msg
    RestrictionUserTypeList.msg
    RoadLaneSetList.msg
    RoadRegulatorID.msg
    RoadSegment.msg
    RoadSegmentID.msg
    RoadSegmentList.msg
    RoadSegmentReferenceID.msg
    RoadwayCrownAngle.msg
    SPAT.msg
    Scale_B12.msg
    SegmentAttributeXY.msg
    SegmentAttributeXYList.msg
    SemiMajorAxisAccuracy.msg
    SemiMajorAxisOrientation.msg
    SemiMinorAxisAccuracy.msg
    SignalGroupID.msg
    SignalRequest.msg
    SignalRequestList.msg
    SignalRequestMessage.msg
    SignalRequestPackage.msg
    SignalRequesterInfo.msg
    SignalStatus.msg
    SignalStatusList.msg
    SignalStatusMessage.msg
    SignalStatusPackage.msg
    SignalStatusPackageList.msg
    SpeedAdvice.msg
    SpeedConfidenceDSRC.msg
    SpeedLimitList.msg
    SpeedLimitType.msg
    SpeedandHeadingandThrottleConfidence.msg
    TemporaryID.msg
    ThrottleConfidence.msg
    TimeChangeDetails.msg
    TimeConfidence.msg
    TimeIntervalConfidence.msg
    TimeMark.msg
    TransitVehicleOccupancy.msg
    TransitVehicleStatus.msg
    TransmissionAndSpeed.msg
    TransmissionState.msg
    VehicleHeight.msg
    VehicleID.msg
    VehicleType.msg
    Velocity.msg
    WaitOnStopline.msg
    ZoneLength.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    its_container_v2_its_container
    dsrc_v2_electronicregistrationidentificationvehicledatamodule
)

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs its_container_v2_its_container dsrc_v2_electronicregistrationidentificationvehicledatamodule
    # CATKIN_DEPENDS message_runtime roscpp
)

add_library(dsrc_v2_dsrc INTERFACE)

add_dependencies(dsrc_v2_dsrc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS dsrc_v2_dsrc FILE dsrc_v2_dsrc.cmake)

