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
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    startTime TimeMark OPTIONAL ,
#    minEndTime TimeMark ,
#    maxEndTime TimeMark OPTIONAL ,
#    likelyTime TimeMark OPTIONAL ,
#    confidence TimeIntervalConfidence OPTIONAL ,
#    nextTime TimeMark OPTIONAL  
# } 
#
# ********** START of TimeChangeDetails **********
#

bool                   startTimePresent
bool                   maxEndTimePresent
bool                   likelyTimePresent
bool                   confidencePresent
bool                   nextTimePresent

TimeMark               startTime         #  When this phase 1st started 
                                         # 
                                         #  Tenths of a second in the current or next hour 
                                         #  In units of 1/10th second from UTC time 
                                         #  A range of 0~36000 covers one hour 
                                         #  The values 35991..36000 are used when a leap second occurs 
                                         #  The value 36001 is used to indicate time >3600 seconds 
                                         #  36002 is to be used when value undefined or unknown 
                                         #  Note that this is NOT expressed in GPS time 
                                         #  or in local time 
                                         # OPTIONAL
TimeMark               minEndTime        #  Expected shortest end time 
                                         # 
                                         #  Tenths of a second in the current or next hour 
                                         #  In units of 1/10th second from UTC time 
                                         #  A range of 0~36000 covers one hour 
                                         #  The values 35991..36000 are used when a leap second occurs 
                                         #  The value 36001 is used to indicate time >3600 seconds 
                                         #  36002 is to be used when value undefined or unknown 
                                         #  Note that this is NOT expressed in GPS time 
                                         #  or in local time 
TimeMark               maxEndTime        #  Expected longest end time 
                                         # 
                                         #  Tenths of a second in the current or next hour 
                                         #  In units of 1/10th second from UTC time 
                                         #  A range of 0~36000 covers one hour 
                                         #  The values 35991..36000 are used when a leap second occurs 
                                         #  The value 36001 is used to indicate time >3600 seconds 
                                         #  36002 is to be used when value undefined or unknown 
                                         #  Note that this is NOT expressed in GPS time 
                                         #  or in local time 
                                         # OPTIONAL
TimeMark               likelyTime        #  Best predicted value based on other data 
                                         # 
                                         #  Tenths of a second in the current or next hour 
                                         #  In units of 1/10th second from UTC time 
                                         #  A range of 0~36000 covers one hour 
                                         #  The values 35991..36000 are used when a leap second occurs 
                                         #  The value 36001 is used to indicate time >3600 seconds 
                                         #  36002 is to be used when value undefined or unknown 
                                         #  Note that this is NOT expressed in GPS time 
                                         #  or in local time 
                                         # OPTIONAL
TimeIntervalConfidence confidence        #  Applies to above time element only 
                                         # OPTIONAL
TimeMark               nextTime          #  A rough estimate of time when  
                                         #  this phase may next occur again 
                                         #  used to support various ECO driving power  
                                         #  management needs. 
                                         # 
                                         #  Tenths of a second in the current or next hour 
                                         #  In units of 1/10th second from UTC time 
                                         #  A range of 0~36000 covers one hour 
                                         #  The values 35991..36000 are used when a leap second occurs 
                                         #  The value 36001 is used to indicate time >3600 seconds 
                                         #  36002 is to be used when value undefined or unknown 
                                         #  Note that this is NOT expressed in GPS time 
                                         #  or in local time 
                                         # OPTIONAL

# ********** END of TimeChangeDetails   **********
