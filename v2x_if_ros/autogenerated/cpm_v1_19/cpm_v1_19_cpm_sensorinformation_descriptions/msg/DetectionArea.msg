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
# Element type: CHOICE
# 
# ASN.1 Definition (Copyright 2019 ETSI):
# CHOICE {
#    vehicleSensor VehicleSensor,
#    stationarySensorRadial AreaRadial,
#    stationarySensorPolygon AreaPolygon,
#    stationarySensorCircular AreaCircular,
#    stationarySensorEllipse AreaEllipse,
#    stationarySensorRectangle AreaRectangle,
#    ... 
# } 
#
# ********** START of DetectionArea **********
#

uint8                                                    choice                    # Use this field to indicate which of the following fields contains valid data.

VehicleSensor                                            vehicleSensor             # choice = 0
cpm_v1_19_cpm_commondatatypes_descriptions/AreaRadial    stationarySensorRadial    # choice = 1
cpm_v1_19_cpm_commondatatypes_descriptions/AreaPolygon   stationarySensorPolygon   # choice = 2
cpm_v1_19_cpm_commondatatypes_descriptions/AreaCircular  stationarySensorCircular  # choice = 3
cpm_v1_19_cpm_commondatatypes_descriptions/AreaEllipse   stationarySensorEllipse   # choice = 4
cpm_v1_19_cpm_commondatatypes_descriptions/AreaRectangle stationarySensorRectangle # choice = 5

# ********** END of DetectionArea   **********
