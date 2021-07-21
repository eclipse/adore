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
# ENUMERATED { unavailable(0), elev_500_00(1), elev_200_00(2), elev_100_00(3), elev_050_00(4), elev_020_00(5), elev_010_00(6), elev_005_00(7), elev_002_00(8), elev_001_00(9), elev_000_50(10), elev_000_20(11), elev_000_10(12), elev_000_05(13), elev_000_02(14), elev_000_01(15) } 
#
# ********** START of ElevationConfidence **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 15
int8            UNAVAILABLE     = 0

int8            ELEV__500_00    = 1
int8            ELEV__200_00    = 2
int8            ELEV__100_00    = 3
int8            ELEV__050_00    = 4
int8            ELEV__020_00    = 5
int8            ELEV__010_00    = 6
int8            ELEV__005_00    = 7
int8            ELEV__002_00    = 8
int8            ELEV__001_00    = 9
int8            ELEV__000_50    = 10
int8            ELEV__000_20    = 11
int8            ELEV__000_10    = 12
int8            ELEV__000_05    = 13
int8            ELEV__000_02    = 14
int8            ELEV__000_01    = 15

# ********** END of ElevationConfidence   **********
