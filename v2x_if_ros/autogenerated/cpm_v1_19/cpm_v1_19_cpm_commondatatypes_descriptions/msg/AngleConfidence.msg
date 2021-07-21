#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                             *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                                         *
#*                                                                                                       *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                              *
#*                                                                                                       *
#* This program and the accompanying materials are made available under the                              *
#* terms of the Eclipse Public License 2.0 which is available at                                         *
#* http://www.eclipse.org/legal/epl-2.0.                                                                 *
#*                                                                                                       *
#* SPDX-License-Identifier: EPL-2.0                                                                      *
#*                                                                                                       *
#*                                                                                                       *
#* File automatically generated with DLR Wind v2 (2021)                                                  *
#*                                                                                                       *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3                                 *
#*                                                                                                       *
#* Module: CPM_SensorInformation_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(103324) sensorInformation(4) version1(1)}                                               *
#*                                                                                                       *
#* For support contact v2x-ts@dlr.de                                                                     *
#*                                                                                                       *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition (Copyright 2019 ETSI):
# INTEGER { zeroPointOneDegree(1), oneDegree(10), outOfRange(126), unavailable(127) }(1..127) 
#
# ********** START of AngleConfidence **********
#

int8            value

int8            MIN                   = 1
int8            MAX                   = 127
int8            UNAVAILABLE           = 127
int8            OUT_OF_RANGE          = 126

int8            ZERO_POINT_ONE_DEGREE = 1
int8            ONE_DEGREE            = 10

# ********** END of AngleConfidence   **********
