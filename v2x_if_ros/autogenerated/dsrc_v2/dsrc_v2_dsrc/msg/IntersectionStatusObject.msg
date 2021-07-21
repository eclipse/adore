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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { manualControlIsEnabled(0), stopTimeIsActivated(1), failureFlash(2), preemptIsActive(3), signalPriorityIsActive(4), fixedTimeOperation(5), trafficDependentOperation(6), standbyOperation(7), failureMode(8), off(9), recentMAPmessageUpdate(10), recentChangeInMAPassignedLanesIDsUsed(11), noValidMAPisAvailableAtThisTime(12), noValidSPATisAvailableAtThisTime(13) }(SIZE(16)) 
#
# ********** START of IntersectionStatusObject **********
#

bool[]          values

uint8           ARRAY_SIZE                                    = 16

uint8           MANUAL_CONTROL_IS_ENABLED                     = 0
uint8           STOP_TIME_IS_ACTIVATED                        = 1
uint8           FAILURE_FLASH                                 = 2
uint8           PREEMPT_IS_ACTIVE                             = 3
uint8           SIGNAL_PRIORITY_IS_ACTIVE                     = 4
uint8           FIXED_TIME_OPERATION                          = 5
uint8           TRAFFIC_DEPENDENT_OPERATION                   = 6
uint8           STANDBY_OPERATION                             = 7
uint8           FAILURE_MODE                                  = 8
uint8           OFF                                           = 9
uint8           RECENT_MA_PMESSAGE_UPDATE                     = 10
uint8           RECENT_CHANGE_IN_MA_PASSIGNED_LANES_I_DS_USED = 11
uint8           NO_VALID_MA_PIS_AVAILABLE_AT_THIS_TIME        = 12
uint8           NO_VALID_SPA_TIS_AVAILABLE_AT_THIS_TIME       = 13
uint8           RESERVED__00                                  = 14
uint8           RESERVED__01                                  = 15

# ********** END of IntersectionStatusObject   **********
