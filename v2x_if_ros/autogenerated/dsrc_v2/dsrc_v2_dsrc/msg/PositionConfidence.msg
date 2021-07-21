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
# ENUMERATED { unavailable(0), a500m(1), a200m(2), a100m(3), a50m(4), a20m(5), a10m(6), a5m(7), a2m(8), a1m(9), a50cm(10), a20cm(11), a10cm(12), a5cm(13), a2cm(14), a1cm(15) } 
#
# ********** START of PositionConfidence **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 15
int8            UNAVAILABLE     = 0

int8            A_500M          = 1
int8            A_200M          = 2
int8            A_100M          = 3
int8            A_50M           = 4
int8            A_20M           = 5
int8            A_10M           = 6
int8            A_5M            = 7
int8            A_2M            = 8
int8            A_1M            = 9
int8            A_50CM          = 10
int8            A_20CM          = 11
int8            A_10CM          = 12
int8            A_5CM           = 13
int8            A_2CM           = 14
int8            A_1CM           = 15

# ********** END of PositionConfidence   **********
