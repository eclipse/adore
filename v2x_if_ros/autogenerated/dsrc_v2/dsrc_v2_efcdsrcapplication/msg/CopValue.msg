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
# ENUMERATED { noEntry(0), co2class1(1), co2class2(2), co2class3(3), co2class4(4), co2class5(5), co2class6(6), co2class7(7), reservedforUse(8) } 
#
# ********** START of CopValue **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 8

int8            NO_ENTRY        = 0
int8            CO_2CLASS_1     = 1
int8            CO_2CLASS_2     = 2
int8            CO_2CLASS_3     = 3
int8            CO_2CLASS_4     = 4
int8            CO_2CLASS_5     = 5
int8            CO_2CLASS_6     = 6
int8            CO_2CLASS_7     = 7
int8            RESERVEDFOR_USE = 8

# ********** END of CopValue   **********
