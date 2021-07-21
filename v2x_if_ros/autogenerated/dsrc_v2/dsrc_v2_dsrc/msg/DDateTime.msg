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
#    year DYear OPTIONAL,
#    month DMonth OPTIONAL,
#    day DDay OPTIONAL,
#    hour DHour OPTIONAL,
#    minute DMinute OPTIONAL,
#    second DSecond OPTIONAL,
#    offset DOffset OPTIONAL 
# } 
#
# ********** START of DDateTime **********
#

bool            yearPresent
bool            monthPresent
bool            dayPresent
bool            hourPresent
bool            minutePresent
bool            secondPresent
bool            offsetPresent

DYear           year            # OPTIONAL
DMonth          month           # OPTIONAL
DDay            day             # OPTIONAL
DHour           hour            # OPTIONAL
DMinute         minute          # OPTIONAL
DSecond         second          # OPTIONAL
DOffset         offset          # OPTIONAL

# ********** END of DDateTime   **********
