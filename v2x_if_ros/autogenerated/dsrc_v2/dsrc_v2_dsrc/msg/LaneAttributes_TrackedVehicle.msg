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
# BIT STRING { spec_RevocableLane(0), spec_commuterRailRoadTrack(1), spec_lightRailRoadTrack(2), spec_heavyRailRoadTrack(3), spec_otherRailType(4) }(SIZE(16)) 
#
# ********** START of LaneAttributes_TrackedVehicle **********
#

bool[]          values

uint8           ARRAY_SIZE                     = 16

uint8           SPEC___REVOCABLE_LANE          = 0
uint8           SPEC__COMMUTER_RAIL_ROAD_TRACK = 1
uint8           SPEC__LIGHT_RAIL_ROAD_TRACK    = 2
uint8           SPEC__HEAVY_RAIL_ROAD_TRACK    = 3
uint8           SPEC__OTHER_RAIL_TYPE          = 4
uint8           RESERVED__00                   = 5
uint8           RESERVED__01                   = 6
uint8           RESERVED__02                   = 7
uint8           RESERVED__03                   = 8
uint8           RESERVED__04                   = 9
uint8           RESERVED__05                   = 10
uint8           RESERVED__06                   = 11
uint8           RESERVED__07                   = 12
uint8           RESERVED__08                   = 13
uint8           RESERVED__09                   = 14
uint8           RESERVED__10                   = 15

# ********** END of LaneAttributes_TrackedVehicle   **********
