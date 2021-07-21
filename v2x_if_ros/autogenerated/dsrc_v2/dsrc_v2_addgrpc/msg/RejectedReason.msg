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
# ENUMERATED { unknown , exceptionalCondition , maxWaitingTimeExceeded , ptPriorityDisabled , higherPTPriorityGranted , vehicleTrackingUnknown , ... } 
#
# ********** START of RejectedReason **********
#

int8            value

int8            MIN                        = 0
int8            MAX                        = 5

int8            UNKNOWN                    = 0
int8            EXCEPTIONAL_CONDITION      = 1
int8            MAX_WAITING_TIME_EXCEEDED  = 2
int8            PT_PRIORITY_DISABLED       = 3
int8            HIGHER_PT_PRIORITY_GRANTED = 4
int8            VEHICLE_TRACKING_UNKNOWN   = 5

# ********** END of RejectedReason   **********
