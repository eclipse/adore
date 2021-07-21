#* Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)        *
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
#* File automatically generated with DLR Wind (07.10.2020 16:24:31)                 *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v1:1.2     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(1)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER { equalOrWithinOneCentimeterPerSec(1), equalOrWithinOneMeterPerSec(100), outOfRange(126), unavailable(127) }(1..127) SD(100) 
#
# ********** START of SpeedConfidence **********
#

float32         value

float32         MIN             = 0.01
float32         MAX             = 1.27
float32         UNAVAILABLE     = 1.27
float32         OUT_OF_RANGE    = 1.26
int8            SCALEDIVISOR    = 100

# ********** END of SpeedConfidence   **********
