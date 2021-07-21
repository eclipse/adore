#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                             *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                                         *
#*                                                                                                       *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                              *
#*                                                                                                       *
#* This program and the accompanying materials are made available under the                              *
#* terms of the Eclipse Public License 2.0 which is available at                                         *
#* http://www.eclipse.org/legal/epl-2.0.                                                                 *
#*                                                                                                       *
#* SPDX-License-Identifier: EPL-2.0                                                                      *
#*                                                                                                       *
#*                                                                                                       *
#* File automatically generated with DLR Wind v2 (2021)                                                  *
#*                                                                                                       *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3                                 *
#*                                                                                                       *
#* Module: CPM_SensorInformation_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(103324) sensorInformation(4) version1(1)}                                               *
#*                                                                                                       *
#* For support contact v2x-ts@dlr.de                                                                     *
#*                                                                                                       *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition (Copyright 2019 ETSI):
# SEQUENCE {
#    heading Heading,
#    speed Speed,
#    vehicleOrientationAngle WGS84Angle OPTIONAL,
#    driveDirection DriveDirection DEFAULT forward,
#    longitudinalAcceleration LongitudinalAcceleration OPTIONAL,
#    lateralAcceleration LateralAcceleration OPTIONAL,
#    verticalAcceleration VerticalAcceleration OPTIONAL,
#    yawRate YawRate OPTIONAL,
#    pitchAngle CartesianAngle OPTIONAL,
#    rollAngle CartesianAngle OPTIONAL,
#    vehicleLength VehicleLength OPTIONAL,
#    vehicleWidth VehicleWidth OPTIONAL,
#    vehicleHeight VehicleHeight OPTIONAL,
#    trailerDataContainer TrailerDataContainer OPTIONAL,
#    ... 
# } 
#
# ********** START of OriginatingVehicleContainer **********
#

bool                                                            vehicleOrientationAnglePresent
bool                                                            longitudinalAccelerationPresent
bool                                                            lateralAccelerationPresent
bool                                                            verticalAccelerationPresent
bool                                                            yawRatePresent
bool                                                            pitchAnglePresent
bool                                                            rollAnglePresent
bool                                                            vehicleLengthPresent
bool                                                            vehicleWidthPresent
bool                                                            vehicleHeightPresent
bool                                                            trailerDataContainerPresent

its_container_v2_its_container/Heading                          heading
its_container_v2_its_container/Speed                            speed
cpm_v1_19_cpm_commondatatypes_descriptions/WGS84Angle           vehicleOrientationAngle         # OPTIONAL
its_container_v2_its_container/DriveDirection                   driveDirection
its_container_v2_its_container/LongitudinalAcceleration         longitudinalAcceleration        # OPTIONAL
its_container_v2_its_container/LateralAcceleration              lateralAcceleration             # OPTIONAL
its_container_v2_its_container/VerticalAcceleration             verticalAcceleration            # OPTIONAL
its_container_v2_its_container/YawRate                          yawRate                         # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/CartesianAngle       pitchAngle                      # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/CartesianAngle       rollAngle                       # OPTIONAL
its_container_v2_its_container/VehicleLength                    vehicleLength                   # OPTIONAL
its_container_v2_its_container/VehicleWidth                     vehicleWidth                    # OPTIONAL
dsrc_v2_dsrc/VehicleHeight                                      vehicleHeight                   # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/TrailerDataContainer trailerDataContainer            # OPTIONAL

# ********** END of OriginatingVehicleContainer   **********
