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
# BIT STRING { median_RevocableLane(0), median(1), whiteLineHashing(2), stripedLines(3), doubleStripedLines(4), trafficCones(5), constructionBarrier(6), trafficChannels(7), lowCurbs(8), highCurbs(9) }(SIZE(16)) 
#
# ********** START of LaneAttributes_Barrier **********
#

bool[]          values

uint8           ARRAY_SIZE              = 16

uint8           MEDIAN___REVOCABLE_LANE = 0
uint8           MEDIAN                  = 1
uint8           WHITE_LINE_HASHING      = 2
uint8           STRIPED_LINES           = 3
uint8           DOUBLE_STRIPED_LINES    = 4
uint8           TRAFFIC_CONES           = 5
uint8           CONSTRUCTION_BARRIER    = 6
uint8           TRAFFIC_CHANNELS        = 7
uint8           LOW_CURBS               = 8
uint8           HIGH_CURBS              = 9
uint8           RESERVED__00            = 10
uint8           RESERVED__01            = 11
uint8           RESERVED__02            = 12
uint8           RESERVED__03            = 13
uint8           RESERVED__04            = 14
uint8           RESERVED__05            = 15

# ********** END of LaneAttributes_Barrier   **********
