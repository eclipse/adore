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
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER { straight(0), degSec_000_01ToRight(_1), degSec_000_01ToLeft(1), unavailable(32767) }(-32766..32767) SD(100) 
#
# ********** START of YawRateValue **********
#

float32         value

float32         MIN                      = -327.66
float32         MAX                      = 327.67
float32         UNAVAILABLE              = 327.67
int8            SCALEDIVISOR             = 100

float32         STRAIGHT                 = 0
float32         DEG_SEC__000_01_TO_RIGHT = -1
float32         DEG_SEC__000_01_TO_LEFT  = 1

# ********** END of YawRateValue   **********
