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
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { none(0), unknown(1), special(2), moto(3), car(4), carOther(5), bus(6), axleCnt2(7), axleCnt3(8), axleCnt4(9), axleCnt4Trailer(10), axleCnt5Trailer(11), axleCnt6Trailer(12), axleCnt5MultiTrailer(13), axleCnt6MultiTrailer(14), axleCnt7MultiTrailer(15), ... } 
#
# ********** START of VehicleType **********
#

int8            value

int8            MIN                      = 0
int8            MAX                      = 15

int8            NONE                     = 0
int8            UNKNOWN                  = 1
int8            SPECIAL                  = 2
int8            MOTO                     = 3
int8            CAR                      = 4
int8            CAR_OTHER                = 5
int8            BUS                      = 6
int8            AXLE_CNT_2               = 7
int8            AXLE_CNT_3               = 8
int8            AXLE_CNT_4               = 9
int8            AXLE_CNT_4_TRAILER       = 10
int8            AXLE_CNT_5_TRAILER       = 11
int8            AXLE_CNT_6_TRAILER       = 12
int8            AXLE_CNT_5_MULTI_TRAILER = 13
int8            AXLE_CNT_6_MULTI_TRAILER = 14
int8            AXLE_CNT_7_MULTI_TRAILER = 15

# ********** END of VehicleType   **********
