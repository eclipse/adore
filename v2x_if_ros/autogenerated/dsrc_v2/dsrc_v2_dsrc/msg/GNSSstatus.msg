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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { unavailable(0), isHealthy(1), isMonitored(2), baseStationType(3), aPDOPofUnder5(4), inViewOfUnder5(5), localCorrectionsPresent(6), networkCorrectionsPresent(7) }(SIZE(8)) 
#
# ********** START of GNSSstatus **********
#

bool[]          values

uint8           ARRAY_SIZE                  = 7

uint8           IS_HEALTHY                  = 1
uint8           IS_MONITORED                = 2
uint8           BASE_STATION_TYPE           = 3
uint8           A_PDO_POF_UNDER_5           = 4
uint8           IN_VIEW_OF_UNDER_5          = 5
uint8           LOCAL_CORRECTIONS_PRESENT   = 6
uint8           NETWORK_CORRECTIONS_PRESENT = 7

# ********** END of GNSSstatus   **********
