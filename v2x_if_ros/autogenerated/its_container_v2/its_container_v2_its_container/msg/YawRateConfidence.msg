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
# ENUMERATED { degSec_000_01(0), degSec_000_05(1), degSec_000_10(2), degSec_001_00(3), degSec_005_00(4), degSec_010_00(5), degSec_100_00(6), outOfRange(7), unavailable(8) } 
#
# ********** START of YawRateConfidence **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 6
int8            UNAVAILABLE     = 8
int8            OUT_OF_RANGE    = 7

int8            DEG_SEC__000_01 = 0
int8            DEG_SEC__000_05 = 1
int8            DEG_SEC__000_10 = 2
int8            DEG_SEC__001_00 = 3
int8            DEG_SEC__005_00 = 4
int8            DEG_SEC__010_00 = 5
int8            DEG_SEC__100_00 = 6

# ********** END of YawRateConfidence   **********
