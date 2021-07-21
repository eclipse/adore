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
# ENUMERATED { unavailable(0), prec10deg(1), prec05deg(2), prec01deg(3), prec0_1deg(4), prec0_05deg(5), prec0_01deg(6), prec0_0125deg(7) } 
#
# ********** START of HeadingConfidenceDSRC **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 7
int8            UNAVAILABLE     = 0

int8            PREC_10DEG      = 1
int8            PREC_05DEG      = 2
int8            PREC_01DEG      = 3
int8            PREC_0_1DEG     = 4
int8            PREC_0_05DEG    = 5
int8            PREC_0_01DEG    = 6
int8            PREC_0_0125DEG  = 7

# ********** END of HeadingConfidenceDSRC   **********
