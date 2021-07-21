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
# This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.4
# 
# Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
#     wg1(1) ts(102894) cdd(2) version(2)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(its_container_v2_its_container)

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
    AccelerationConfidence.msg
    AccelerationControl.msg
    AccidentSubCauseCode.msg
    ActionID.msg
    AdverseWeatherCondition_AdhesionSubCauseCode.msg
    AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode.msg
    AdverseWeatherCondition_PrecipitationSubCauseCode.msg
    AdverseWeatherCondition_VisibilitySubCauseCode.msg
    Altitude.msg
    AltitudeConfidence.msg
    AltitudeValue.msg
    CauseCode.msg
    CauseCodeType.msg
    CenDsrcTollingZone.msg
    CenDsrcTollingZoneID.msg
    ClosedLanes.msg
    CollisionRiskSubCauseCode.msg
    Curvature.msg
    CurvatureCalculationMode.msg
    CurvatureConfidence.msg
    CurvatureValue.msg
    DangerousEndOfQueueSubCauseCode.msg
    DangerousGoodsBasic.msg
    DangerousGoodsExtended.msg
    DangerousGoodsExtended_companyName.msg
    DangerousGoodsExtended_elevatedTemperature.msg
    DangerousGoodsExtended_emergencyActionCode.msg
    DangerousGoodsExtended_limitedQuantity.msg
    DangerousGoodsExtended_tunnelsRestricted.msg
    DangerousGoodsExtended_unNumber.msg
    DangerousSituationSubCauseCode.msg
    DeltaAltitude.msg
    DeltaLatitude.msg
    DeltaLongitude.msg
    DeltaReferencePosition.msg
    DigitalMap.msg
    DriveDirection.msg
    DrivingLaneStatus.msg
    EmbarkationStatus.msg
    EmergencyPriority.msg
    EmergencyVehicleApproachingSubCauseCode.msg
    EnergyStorageType.msg
    EventHistory.msg
    EventPoint.msg
    ExteriorLights.msg
    HardShoulderStatus.msg
    HazardousLocation_AnimalOnTheRoadSubCauseCode.msg
    HazardousLocation_DangerousCurveSubCauseCode.msg
    HazardousLocation_ObstacleOnTheRoadSubCauseCode.msg
    HazardousLocation_SurfaceConditionSubCauseCode.msg
    Heading.msg
    HeadingConfidence.msg
    HeadingValue.msg
    HeightLonCarr.msg
    HumanPresenceOnTheRoadSubCauseCode.msg
    HumanProblemSubCauseCode.msg
    InformationQuality.msg
    ItineraryPath.msg
    ItsPduHeader.msg
    ItsPduHeader_messageID.msg
    ItsPduHeader_protocolVersion.msg
    LanePosition.msg
    LateralAcceleration.msg
    LateralAccelerationValue.msg
    Latitude.msg
    LightBarSirenInUse.msg
    Longitude.msg
    LongitudinalAcceleration.msg
    LongitudinalAccelerationValue.msg
    NumberOfOccupants.msg
    PathDeltaTime.msg
    PathHistory.msg
    PathPoint.msg
    PerformanceClass.msg
    PhoneNumber.msg
    PosCentMass.msg
    PosConfidenceEllipse.msg
    PosFrontAx.msg
    PosLonCarr.msg
    PosPillar.msg
    PositionOfOccupants.msg
    PositionOfPillars.msg
    PositioningSolutionType.msg
    PostCrashSubCauseCode.msg
    ProtectedCommunicationZone.msg
    ProtectedCommunicationZonesRSU.msg
    ProtectedZoneID.msg
    ProtectedZoneRadius.msg
    ProtectedZoneType.msg
    PtActivation.msg
    PtActivationData.msg
    PtActivationType.msg
    ReferencePosition.msg
    RelevanceDistance.msg
    RelevanceTrafficDirection.msg
    RequestResponseIndication.msg
    RescueAndRecoveryWorkInProgressSubCauseCode.msg
    RestrictedTypes.msg
    RoadType.msg
    RoadworksSubCauseCode.msg
    SemiAxisLength.msg
    SequenceNumber.msg
    SignalViolationSubCauseCode.msg
    SlowVehicleSubCauseCode.msg
    SpecialTransportType.msg
    Speed.msg
    SpeedConfidence.msg
    SpeedLimit.msg
    SpeedValue.msg
    StationID.msg
    StationType.msg
    StationarySince.msg
    StationaryVehicleSubCauseCode.msg
    SteeringWheelAngle.msg
    SteeringWheelAngleConfidence.msg
    SteeringWheelAngleValue.msg
    SubCauseCodeType.msg
    Temperature.msg
    TimestampIts.msg
    Traces.msg
    TrafficConditionSubCauseCode.msg
    TrafficRule.msg
    TransmissionInterval.msg
    TurningRadius.msg
    VDS.msg
    ValidityDuration.msg
    VehicleBreakdownSubCauseCode.msg
    VehicleIdentification.msg
    VehicleLength.msg
    VehicleLengthConfidenceIndication.msg
    VehicleLengthValue.msg
    VehicleMass.msg
    VehicleRole.msg
    VehicleWidth.msg
    VerticalAcceleration.msg
    VerticalAccelerationValue.msg
    WMInumber.msg
    WheelBaseVehicle.msg
    WrongWayDrivingSubCauseCode.msg
    YawRate.msg
    YawRateConfidence.msg
    YawRateValue.msg
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

add_library(its_container_v2_its_container INTERFACE)

add_dependencies(its_container_v2_its_container ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

export(TARGETS its_container_v2_its_container FILE its_container_v2_its_container.cmake)

