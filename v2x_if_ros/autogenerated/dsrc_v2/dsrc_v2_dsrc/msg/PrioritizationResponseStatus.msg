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
# ENUMERATED { unknown(0), requested(1), processing(2), watchOtherTraffic(3), granted(4), rejected(5), maxPresence(6), reserviceLocked(7), ... } 
#
# ********** START of PrioritizationResponseStatus **********
#

int8            value

int8            MIN                 = 0
int8            MAX                 = 7

int8            UNKNOWN             = 0
int8            REQUESTED           = 1
int8            PROCESSING          = 2
int8            WATCH_OTHER_TRAFFIC = 3
int8            GRANTED             = 4
int8            REJECTED            = 5
int8            MAX_PRESENCE        = 6
int8            RESERVICE_LOCKED    = 7

# ********** END of PrioritizationResponseStatus   **********
