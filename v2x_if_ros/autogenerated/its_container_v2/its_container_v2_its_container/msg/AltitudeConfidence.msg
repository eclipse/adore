#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)        *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                    *
#*                                                                                  *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore         *
#*                                                                                  *
#* This program and the accompanying materials are made available under the         *
#* terms of the Eclipse Public License 2.0 which is available at                    *
#* http://www.eclipse.org/legal/epl-2.0.                                            *
#*                                                                                  *
#* SPDX-License-Identifier: EPL-2.0                                                 *
#*                                                                                  *
#*                                                                                  *
#* File automatically generated with DLR Wind v2 (2021)                             *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.4     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(2)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { alt_000_01(0), alt_000_02(1), alt_000_05(2), alt_000_10(3), alt_000_20(4), alt_000_50(5), alt_001_00(6), alt_002_00(7), alt_005_00(8), alt_010_00(9), alt_020_00(10), alt_050_00(11), alt_100_00(12), alt_200_00(13), outOfRange(14), unavailable(15) } 
#
# ********** START of AltitudeConfidence **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 13
int8            UNAVAILABLE     = 15
int8            OUT_OF_RANGE    = 14

int8            ALT__000_01     = 0
int8            ALT__000_02     = 1
int8            ALT__000_05     = 2
int8            ALT__000_10     = 3
int8            ALT__000_20     = 4
int8            ALT__000_50     = 5
int8            ALT__001_00     = 6
int8            ALT__002_00     = 7
int8            ALT__005_00     = 8
int8            ALT__010_00     = 9
int8            ALT__020_00     = 10
int8            ALT__050_00     = 11
int8            ALT__100_00     = 12
int8            ALT__200_00     = 13

# ********** END of AltitudeConfidence   **********
