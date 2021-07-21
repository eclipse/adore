#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)    *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                *
#*                                                                              *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore     *
#*                                                                              *
#* This program and the accompanying materials are made available under the     *
#* terms of the Eclipse Public License 2.0 which is available at                *
#* http://www.eclipse.org/legal/epl-2.0.                                        *
#*                                                                              *
#* SPDX-License-Identifier: EPL-2.0                                             *
#*                                                                              *
#*                                                                              *
#* File automatically generated with DLR Wind v2 (2021)                         *
#*                                                                              *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0        *
#*                                                                              *
#* Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(302637) mcm(1) version(3)}                                     *
#*                                                                              *
#* For support contact v2x-ts@dlr.de                                            *
#*                                                                              *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    vehicleCapabilities VehicleCapabilities,
#    plannedTrajectory PlannedTrajectory,
#    desiredTrajectory DesiredTrajectory OPTIONAL,
#    respectedDesiredTrajectoriesList RespectedDesiredTrajectoriesList,
#    triggerTimeOfToC TriggerTimeOfToC OPTIONAL,
#    targetAutomationLevel TargetAutomationLevel OPTIONAL,
#    triggerTimeOfMRM TriggerTimeOfMRM OPTIONAL,
#    heading Heading,
#    speed Speed,
#    vehicleLength VehicleLength,
#    vehicleWidth VehicleWidth,
#    longitudinalAcceleration LongitudinalAcceleration,
#    lateralAcceleration LateralAcceleration,
#    verticalAcceleration VerticalAcceleration,
#    yawRate YawRate,
#    curvature Curvature,
#    curvatureCalculationMode CurvatureCalculationMode,
#    driveDirection DriveDirection,
#    lanePosition LanePosition,
#    steeringWheelAngle SteeringWheelAngle,
#    adviceResponseList AdviceResponseList OPTIONAL 
# } 
#
# ********** START of VehicleManeuver **********
#

bool                                                    desiredTrajectoryPresent
bool                                                    triggerTimeOfToCPresent
bool                                                    targetAutomationLevelPresent
bool                                                    triggerTimeOfMRMPresent
bool                                                    adviceResponseListPresent

VehicleCapabilities                                     vehicleCapabilities
PlannedTrajectory                                       plannedTrajectory
DesiredTrajectory                                       desiredTrajectory                # OPTIONAL
RespectedDesiredTrajectoriesList                        respectedDesiredTrajectoriesList
TriggerTimeOfToC                                        triggerTimeOfToC                 # OPTIONAL
TargetAutomationLevel                                   targetAutomationLevel            # OPTIONAL
TriggerTimeOfMRM                                        triggerTimeOfMRM                 # OPTIONAL
its_container_v2_its_container/Heading                  heading
its_container_v2_its_container/Speed                    speed
its_container_v2_its_container/VehicleLength            vehicleLength
its_container_v2_its_container/VehicleWidth             vehicleWidth
its_container_v2_its_container/LongitudinalAcceleration longitudinalAcceleration
its_container_v2_its_container/LateralAcceleration      lateralAcceleration
its_container_v2_its_container/VerticalAcceleration     verticalAcceleration
its_container_v2_its_container/YawRate                  yawRate
its_container_v2_its_container/Curvature                curvature
its_container_v2_its_container/CurvatureCalculationMode curvatureCalculationMode
its_container_v2_its_container/DriveDirection           driveDirection
its_container_v2_its_container/LanePosition             lanePosition
its_container_v2_its_container/SteeringWheelAngle       steeringWheelAngle
AdviceResponseList                                      adviceResponseList               # OPTIONAL

# ********** END of VehicleManeuver   **********
