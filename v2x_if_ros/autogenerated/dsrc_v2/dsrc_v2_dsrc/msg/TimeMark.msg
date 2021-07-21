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
#  Tenths of a second in the current or next hour 
#  In units of 1/10th second from UTC time 
#  A range of 0~36000 covers one hour 
#  The values 35991..36000 are used when a leap second occurs 
#  The value 36001 is used to indicate time >3600 seconds 
#  36002 is to be used when value undefined or unknown 
#  Note that this is NOT expressed in GPS time 
#  or in local time 
# 
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER(0..36001)  
#
# ********** START of TimeMark **********
#

uint16          value

uint16          MIN             = 0
uint16          MAX             = 36001


# ********** END of TimeMark   **********
