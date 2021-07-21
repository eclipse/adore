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
# INTEGER { mid(0), bottomLeft(1), midLeft(2), topLeft(3), bottomMid(4), topMid(5), bottomRight(6), midRight(7), topRight(8) }(0..8) 
#
# ********** START of ObjectRefPoint **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 8

int8            MID             = 0
int8            BOTTOM_LEFT     = 1
int8            MID_LEFT        = 2
int8            TOP_LEFT        = 3
int8            BOTTOM_MID      = 4
int8            TOP_MID         = 5
int8            BOTTOM_RIGHT    = 6
int8            MID_RIGHT       = 7
int8            TOP_RIGHT       = 8

# ********** END of ObjectRefPoint   **********
