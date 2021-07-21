#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                  *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                              *
#*                                                                                            *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                   *
#*                                                                                            *
#* This program and the accompanying materials are made available under the                   *
#* terms of the Eclipse Public License 2.0 which is available at                              *
#* http://www.eclipse.org/legal/epl-2.0.                                                      *
#*                                                                                            *
#* SPDX-License-Identifier: EPL-2.0                                                           *
#*                                                                                            *
#*                                                                                            *
#* File automatically generated with DLR Wind v2 (2021)                                       *
#*                                                                                            *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:dsrc_v2:2.5                        *
#*                                                                                            *
#* Module: RTCMEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(103301) rtcmem(5) version1(1)}                                               *
#*                                                                                            *
#* For support contact v2x-ts@dlr.de                                                          *
#*                                                                                            *

#
# Element type: CHOICE
# 
# ASN.1 Definition:
# CHOICE {
#    vehicle LaneAttributes_Vehicle,
#    crosswalk LaneAttributes_Crosswalk,
#    bikeLane LaneAttributes_Bike,
#    sidewalk LaneAttributes_Sidewalk,
#    median LaneAttributes_Barrier,
#    striping LaneAttributes_Striping,
#    trackedVehicle LaneAttributes_TrackedVehicle,
#    parking LaneAttributes_Parking,
#    ... 
# } 
#
# ********** START of LaneTypeAttributes **********
#

uint8                         choice          # Use this field to indicate which of the following fields contains valid data.

LaneAttributes_Vehicle        vehicle         # choice = 0
LaneAttributes_Crosswalk      crosswalk       # choice = 1
LaneAttributes_Bike           bikeLane        # choice = 2
LaneAttributes_Sidewalk       sidewalk        # choice = 3
LaneAttributes_Barrier        median          # choice = 4
LaneAttributes_Striping       striping        # choice = 5
LaneAttributes_TrackedVehicle trackedVehicle  # choice = 6
LaneAttributes_Parking        parking         # choice = 7

# ********** END of LaneTypeAttributes   **********
