#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)               *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                           *
#*                                                                                         *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                *
#*                                                                                         *
#* This program and the accompanying materials are made available under the                *
#* terms of the Eclipse Public License 2.0 which is available at                           *
#* http://www.eclipse.org/legal/epl-2.0.                                                   *
#*                                                                                         *
#* SPDX-License-Identifier: EPL-2.0                                                        *
#*                                                                                         *
#*                                                                                         *
#* File automatically generated with DLR Wind v2 (2021)                                    *
#*                                                                                         *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cam_v2:1.4                      *
#*                                                                                         *
#* Module: CAM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(302637) cam(2) version(2)}                                                *
#*                                                                                         *
#* For support contact v2x-ts@dlr.de                                                       *
#*                                                                                         *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    heading Heading,
#    speed Speed,
#    driveDirection DriveDirection,
#    vehicleLength VehicleLength,
#    vehicleWidth VehicleWidth,
#    longitudinalAcceleration LongitudinalAcceleration,
#    curvature Curvature,
#    curvatureCalculationMode CurvatureCalculationMode,
#    yawRate YawRate,
#    accelerationControl AccelerationControl OPTIONAL,
#    lanePosition LanePosition OPTIONAL,
#    steeringWheelAngle SteeringWheelAngle OPTIONAL,
#    lateralAcceleration LateralAcceleration OPTIONAL,
#    verticalAcceleration VerticalAcceleration OPTIONAL,
#    performanceClass PerformanceClass OPTIONAL,
#    cenDsrcTollingZone CenDsrcTollingZone OPTIONAL 
# } 
#
# ********** START of BasicVehicleContainerHighFrequency **********
#

bool                                                    accelerationControlPresent
bool                                                    lanePositionPresent
bool                                                    steeringWheelAnglePresent
bool                                                    lateralAccelerationPresent
bool                                                    verticalAccelerationPresent
bool                                                    performanceClassPresent
bool                                                    cenDsrcTollingZonePresent

its_container_v2_its_container/Heading                  heading
its_container_v2_its_container/Speed                    speed
its_container_v2_its_container/DriveDirection           driveDirection
its_container_v2_its_container/VehicleLength            vehicleLength
its_container_v2_its_container/VehicleWidth             vehicleWidth
its_container_v2_its_container/LongitudinalAcceleration longitudinalAcceleration
its_container_v2_its_container/Curvature                curvature
its_container_v2_its_container/CurvatureCalculationMode curvatureCalculationMode
its_container_v2_its_container/YawRate                  yawRate
its_container_v2_its_container/AccelerationControl      accelerationControl         # OPTIONAL
its_container_v2_its_container/LanePosition             lanePosition                # OPTIONAL
its_container_v2_its_container/SteeringWheelAngle       steeringWheelAngle          # OPTIONAL
its_container_v2_its_container/LateralAcceleration      lateralAcceleration         # OPTIONAL
its_container_v2_its_container/VerticalAcceleration     verticalAcceleration        # OPTIONAL
its_container_v2_its_container/PerformanceClass         performanceClass            # OPTIONAL
its_container_v2_its_container/CenDsrcTollingZone       cenDsrcTollingZone          # OPTIONAL

# ********** END of BasicVehicleContainerHighFrequency   **********
