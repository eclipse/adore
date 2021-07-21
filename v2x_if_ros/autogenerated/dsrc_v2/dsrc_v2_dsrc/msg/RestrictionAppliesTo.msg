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
# ENUMERATED { none , equippedTransit , equippedTaxis , equippedOther , emissionCompliant , equippedBicycle , weightCompliant , heightCompliant , pedestrians , slowMovingPersons , wheelchairUsers , visualDisabilities , audioDisabilities , otherUnknownDisabilities , ... } 
#
# ********** START of RestrictionAppliesTo **********
#

int8            value

int8            MIN                        = 0
int8            MAX                        = 13

int8            NONE                       = 0
int8            EQUIPPED_TRANSIT           = 1
int8            EQUIPPED_TAXIS             = 2
int8            EQUIPPED_OTHER             = 3
int8            EMISSION_COMPLIANT         = 4
int8            EQUIPPED_BICYCLE           = 5
int8            WEIGHT_COMPLIANT           = 6
int8            HEIGHT_COMPLIANT           = 7
int8            PEDESTRIANS                = 8
int8            SLOW_MOVING_PERSONS        = 9
int8            WHEELCHAIR_USERS           = 10
int8            VISUAL_DISABILITIES        = 11
int8            AUDIO_DISABILITIES         = 12
int8            OTHER_UNKNOWN_DISABILITIES = 13

# ********** END of RestrictionAppliesTo   **********
