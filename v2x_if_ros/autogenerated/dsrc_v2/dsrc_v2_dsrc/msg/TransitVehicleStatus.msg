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
# BIT STRING { loading(0), anADAuse(1), aBikeLoad(2), doorOpen(3), charging(4), atStopLine(5) }(SIZE(8)) 
#
# ********** START of TransitVehicleStatus **********
#

bool[]          values

uint8           ARRAY_SIZE      = 8

uint8           LOADING         = 0
uint8           AN_AD_AUSE      = 1
uint8           A_BIKE_LOAD     = 2
uint8           DOOR_OPEN       = 3
uint8           CHARGING        = 4
uint8           AT_STOP_LINE    = 5
uint8           RESERVED__00    = 6
uint8           RESERVED__01    = 7

# ********** END of TransitVehicleStatus   **********
