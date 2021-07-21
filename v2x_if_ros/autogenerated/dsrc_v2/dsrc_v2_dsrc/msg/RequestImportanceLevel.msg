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
# ENUMERATED { requestImportanceLevelUnKnown(0), requestImportanceLevel1(1), requestImportanceLevel2(2), requestImportanceLevel3(3), requestImportanceLevel4(4), requestImportanceLevel5(5), requestImportanceLevel6(6), requestImportanceLevel7(7), requestImportanceLevel8(8), requestImportanceLevel9(9), requestImportanceLevel10(10), requestImportanceLevel11(11), requestImportanceLevel12(12), requestImportanceLevel13(13), requestImportanceLevel14(14), requestImportanceReserved(15) } 
#
# ********** START of RequestImportanceLevel **********
#

int8            value

int8            MIN                               = 0
int8            MAX                               = 15

int8            REQUEST_IMPORTANCE_LEVEL_UN_KNOWN = 0
int8            REQUEST_IMPORTANCE_LEVEL_1        = 1
int8            REQUEST_IMPORTANCE_LEVEL_2        = 2
int8            REQUEST_IMPORTANCE_LEVEL_3        = 3
int8            REQUEST_IMPORTANCE_LEVEL_4        = 4
int8            REQUEST_IMPORTANCE_LEVEL_5        = 5
int8            REQUEST_IMPORTANCE_LEVEL_6        = 6
int8            REQUEST_IMPORTANCE_LEVEL_7        = 7
int8            REQUEST_IMPORTANCE_LEVEL_8        = 8
int8            REQUEST_IMPORTANCE_LEVEL_9        = 9
int8            REQUEST_IMPORTANCE_LEVEL_10       = 10
int8            REQUEST_IMPORTANCE_LEVEL_11       = 11
int8            REQUEST_IMPORTANCE_LEVEL_12       = 12
int8            REQUEST_IMPORTANCE_LEVEL_13       = 13
int8            REQUEST_IMPORTANCE_LEVEL_14       = 14
int8            REQUEST_IMPORTANCE_RESERVED       = 15

# ********** END of RequestImportanceLevel   **********
