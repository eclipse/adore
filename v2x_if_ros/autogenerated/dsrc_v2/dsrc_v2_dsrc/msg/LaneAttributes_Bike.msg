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
# BIT STRING { bikeRevocableLane(0), pedestrianUseAllowed(1), isBikeFlyOverLane(2), fixedCycleTime(3), biDirectionalCycleTimes(4), isolatedByBarrier(5), unsignalizedSegmentsPresent(6) }(SIZE(16)) 
#
# ********** START of LaneAttributes_Bike **********
#

bool[]          values

uint8           ARRAY_SIZE                    = 16

uint8           BIKE_REVOCABLE_LANE           = 0
uint8           PEDESTRIAN_USE_ALLOWED        = 1
uint8           IS_BIKE_FLY_OVER_LANE         = 2
uint8           FIXED_CYCLE_TIME              = 3
uint8           BI_DIRECTIONAL_CYCLE_TIMES    = 4
uint8           ISOLATED_BY_BARRIER           = 5
uint8           UNSIGNALIZED_SEGMENTS_PRESENT = 6
uint8           RESERVED__00                  = 7
uint8           RESERVED__01                  = 8
uint8           RESERVED__02                  = 9
uint8           RESERVED__03                  = 10
uint8           RESERVED__04                  = 11
uint8           RESERVED__05                  = 12
uint8           RESERVED__06                  = 13
uint8           RESERVED__07                  = 14
uint8           RESERVED__08                  = 15

# ********** END of LaneAttributes_Bike   **********
