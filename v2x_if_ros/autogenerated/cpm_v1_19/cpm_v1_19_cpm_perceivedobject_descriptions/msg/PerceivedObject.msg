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
#    objectID Identifier,
#    timeOfMeasurement TimeOfMeasurement,
#    objectConfidence ObjectConfidence DEFAULT 0,
#    xDistance ObjectDistance,
#    yDistance ObjectDistance,
#    zDistance ObjectDistance OPTIONAL,
#    xSpeed SpeedExtended,
#    ySpeed SpeedExtended,
#    zSpeed SpeedExtended OPTIONAL,
#    xAcceleration LongitudinalAcceleration OPTIONAL,
#    yAcceleration LateralAcceleration OPTIONAL,
#    zAcceleration VerticalAcceleration OPTIONAL,
#    yawAngle CartesianAngle OPTIONAL,
#    planarObjectDimension1 ObjectDimension OPTIONAL,
#    planarObjectDimension2 ObjectDimension OPTIONAL,
#    verticalObjectDimension ObjectDimension OPTIONAL,
#    objectRefPoint ObjectRefPoint DEFAULT 0,
#    objectAge ObjectAge OPTIONAL,
#    sensorIDList SensorIdList OPTIONAL,
#    dynamicStatus DynamicStatus OPTIONAL,
#    classification ObjectClassDescription OPTIONAL,
#    matchedPosition MatchedPosition OPTIONAL,
#    ... 
# } 
#
# ********** START of PerceivedObject **********
#

bool                                                              zDistancePresent
bool                                                              zSpeedPresent
bool                                                              xAccelerationPresent
bool                                                              yAccelerationPresent
bool                                                              zAccelerationPresent
bool                                                              yawAnglePresent
bool                                                              planarObjectDimension1Present
bool                                                              planarObjectDimension2Present
bool                                                              verticalObjectDimensionPresent
bool                                                              objectAgePresent
bool                                                              sensorIDListPresent
bool                                                              dynamicStatusPresent
bool                                                              classificationPresent
bool                                                              matchedPositionPresent

cpm_v1_19_cpm_commondatatypes_descriptions/Identifier             objectID
cpm_v1_19_cpm_commondatatypes_descriptions/TimeOfMeasurement      timeOfMeasurement
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectConfidence       objectConfidence
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDistance         xDistance
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDistance         yDistance
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDistance         zDistance                      # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/SpeedExtended          xSpeed
cpm_v1_19_cpm_commondatatypes_descriptions/SpeedExtended          ySpeed
cpm_v1_19_cpm_commondatatypes_descriptions/SpeedExtended          zSpeed                         # OPTIONAL
its_container_v2_its_container/LongitudinalAcceleration           xAcceleration                  # OPTIONAL
its_container_v2_its_container/LateralAcceleration                yAcceleration                  # OPTIONAL
its_container_v2_its_container/VerticalAcceleration               zAcceleration                  # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/CartesianAngle         yawAngle                       # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDimension        planarObjectDimension1         # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDimension        planarObjectDimension2         # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectDimension        verticalObjectDimension        # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectRefPoint         objectRefPoint
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectAge              objectAge                      # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/SensorIdList           sensorIDList                   # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/DynamicStatus          dynamicStatus                  # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/ObjectClassDescription classification                 # OPTIONAL
cpm_v1_19_cpm_commondatatypes_descriptions/MatchedPosition        matchedPosition                # OPTIONAL

# ********** END of PerceivedObject   **********
