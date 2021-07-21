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
# ENUMERATED { reserved , stopLine , roundedCapStyleA , roundedCapStyleB , mergePoint , divergePoint , downstreamStopLine , downstreamStartNode , closedToTraffic , safeIsland , curbPresentAtStepOff , hydrantPresent , ... } 
#
# ********** START of NodeAttributeXY **********
#

int8            value

int8            MIN                      = 0
int8            MAX                      = 11

int8            RESERVED                 = 0
int8            STOP_LINE                = 1
int8            ROUNDED_CAP_STYLE_A      = 2
int8            ROUNDED_CAP_STYLE_B      = 3
int8            MERGE_POINT              = 4
int8            DIVERGE_POINT            = 5
int8            DOWNSTREAM_STOP_LINE     = 6
int8            DOWNSTREAM_START_NODE    = 7
int8            CLOSED_TO_TRAFFIC        = 8
int8            SAFE_ISLAND              = 9
int8            CURB_PRESENT_AT_STEP_OFF = 10
int8            HYDRANT_PRESENT          = 11

# ********** END of NodeAttributeXY   **********
