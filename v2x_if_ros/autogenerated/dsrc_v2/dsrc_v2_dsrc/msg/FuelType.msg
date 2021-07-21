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
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER(0..15) { unknownFuel(0), gasoline(1), ethanol(2), diesel(3), electric(4), hybrid(5), hydrogen(6), natGasLiquid(7), natGasComp(8), propane(9) } 
#
# ********** START of FuelType **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 15

int8            UNKNOWN_FUEL    = 0
int8            GASOLINE        = 1
int8            ETHANOL         = 2
int8            DIESEL          = 3
int8            ELECTRIC        = 4
int8            HYBRID          = 5
int8            HYDROGEN        = 6
int8            NAT_GAS_LIQUID  = 7
int8            NAT_GAS_COMP    = 8
int8            PROPANE         = 9

# ********** END of FuelType   **********
