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
# INTEGER { noEntry(0), noEngine(1), petrolUnleaded(2), petrolLeaded(3), diesel(4), lPG(5), battery(6), solar(7), hybrid(8), hydrogen(9) }(0..255) 
#
# ********** START of EngineCharacteristics **********
#

uint8           value

uint8           MIN             = 0
uint8           MAX             = 255

uint8           NO_ENTRY        = 0
uint8           NO_ENGINE       = 1
uint8           PETROL_UNLEADED = 2
uint8           PETROL_LEADED   = 3
uint8           DIESEL          = 4
uint8           L_PG            = 5
uint8           BATTERY         = 6
uint8           SOLAR           = 7
uint8           HYBRID          = 8
uint8           HYDROGEN        = 9

# ********** END of EngineCharacteristics   **********
