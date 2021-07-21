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
# ENUMERATED { noEntry(0), euro_1(1), euro_2(2), euro_3(3), euro_4(4), euro_5(5), euro_6(6), reservedForUse1(7), reservedForUse2(8), reservedForUse3(9), reservedForUse4(10), reservedForUse5(11), reservedForUse6(12), reservedForUse7(13), reservedForUse8(14), eev(15) } 
#
# ********** START of EuroValue **********
#

int8            value

int8            MIN                = 0
int8            MAX                = 15

int8            NO_ENTRY           = 0
int8            EURO__1            = 1
int8            EURO__2            = 2
int8            EURO__3            = 3
int8            EURO__4            = 4
int8            EURO__5            = 5
int8            EURO__6            = 6
int8            RESERVED_FOR_USE_1 = 7
int8            RESERVED_FOR_USE_2 = 8
int8            RESERVED_FOR_USE_3 = 9
int8            RESERVED_FOR_USE_4 = 10
int8            RESERVED_FOR_USE_5 = 11
int8            RESERVED_FOR_USE_6 = 12
int8            RESERVED_FOR_USE_7 = 13
int8            RESERVED_FOR_USE_8 = 14
int8            EEV                = 15

# ********** END of EuroValue   **********
