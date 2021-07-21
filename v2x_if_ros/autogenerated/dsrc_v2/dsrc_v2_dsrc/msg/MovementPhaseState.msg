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
# ENUMERATED { unavailable(0), dark(1), stop_Then_Proceed(2), stop_And_Remain(3), pre_Movement(4), permissive_Movement_Allowed(5), protected_Movement_Allowed(6), permissive_clearance(7), protected_clearance(8), caution_Conflicting_Traffic(9) } 
#
# ********** START of MovementPhaseState **********
#

int8            value

int8            MIN                             = 0
int8            MAX                             = 9
int8            UNAVAILABLE                     = 0

int8            DARK                            = 1
int8            STOP___THEN___PROCEED           = 2
int8            STOP___AND___REMAIN             = 3
int8            PRE___MOVEMENT                  = 4
int8            PERMISSIVE___MOVEMENT___ALLOWED = 5
int8            PROTECTED___MOVEMENT___ALLOWED  = 6
int8            PERMISSIVE__CLEARANCE           = 7
int8            PROTECTED__CLEARANCE            = 8
int8            CAUTION___CONFLICTING___TRAFFIC = 9

# ********** END of MovementPhaseState   **********
