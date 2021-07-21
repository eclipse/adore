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
# ENUMERATED { requestSubRoleUnKnown(0), requestSubRole1(1), requestSubRole2(2), requestSubRole3(3), requestSubRole4(4), requestSubRole5(5), requestSubRole6(6), requestSubRole7(7), requestSubRole8(8), requestSubRole9(9), requestSubRole10(10), requestSubRole11(11), requestSubRole12(12), requestSubRole13(13), requestSubRole14(14), requestSubRoleReserved(15) } 
#
# ********** START of RequestSubRole **********
#

int8            value

int8            MIN                       = 0
int8            MAX                       = 15

int8            REQUEST_SUB_ROLE_UN_KNOWN = 0
int8            REQUEST_SUB_ROLE_1        = 1
int8            REQUEST_SUB_ROLE_2        = 2
int8            REQUEST_SUB_ROLE_3        = 3
int8            REQUEST_SUB_ROLE_4        = 4
int8            REQUEST_SUB_ROLE_5        = 5
int8            REQUEST_SUB_ROLE_6        = 6
int8            REQUEST_SUB_ROLE_7        = 7
int8            REQUEST_SUB_ROLE_8        = 8
int8            REQUEST_SUB_ROLE_9        = 9
int8            REQUEST_SUB_ROLE_10       = 10
int8            REQUEST_SUB_ROLE_11       = 11
int8            REQUEST_SUB_ROLE_12       = 12
int8            REQUEST_SUB_ROLE_13       = 13
int8            REQUEST_SUB_ROLE_14       = 14
int8            REQUEST_SUB_ROLE_RESERVED = 15

# ********** END of RequestSubRole   **********
