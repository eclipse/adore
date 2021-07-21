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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { isVehicleRevocableLane(0), isVehicleFlyOverLane(1), hovLaneUseOnly(2), restrictedToBusUse(3), restrictedToTaxiUse(4), restrictedFromPublicUse(5), hasIRbeaconCoverage(6), permissionOnRequest(7) }(SIZE(8,...)) 
#
# ********** START of LaneAttributes_Vehicle **********
#

bool[]          values

uint8           ARRAY_SIZE                 = 8

uint8           IS_VEHICLE_REVOCABLE_LANE  = 0
uint8           IS_VEHICLE_FLY_OVER_LANE   = 1
uint8           HOV_LANE_USE_ONLY          = 2
uint8           RESTRICTED_TO_BUS_USE      = 3
uint8           RESTRICTED_TO_TAXI_USE     = 4
uint8           RESTRICTED_FROM_PUBLIC_USE = 5
uint8           HAS_I_RBEACON_COVERAGE     = 6
uint8           PERMISSION_ON_REQUEST      = 7

# ********** END of LaneAttributes_Vehicle   **********
