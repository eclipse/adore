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
# BIT STRING { parkingRevocableLane(0), parallelParkingInUse(1), headInParkingInUse(2), doNotParkZone(3), parkingForBusUse(4), parkingForTaxiUse(5), noPublicParkingUse(6) }(SIZE(16)) 
#
# ********** START of LaneAttributes_Parking **********
#

bool[]          values

uint8           ARRAY_SIZE              = 16

uint8           PARKING_REVOCABLE_LANE  = 0
uint8           PARALLEL_PARKING_IN_USE = 1
uint8           HEAD_IN_PARKING_IN_USE  = 2
uint8           DO_NOT_PARK_ZONE        = 3
uint8           PARKING_FOR_BUS_USE     = 4
uint8           PARKING_FOR_TAXI_USE    = 5
uint8           NO_PUBLIC_PARKING_USE   = 6
uint8           RESERVED__00            = 7
uint8           RESERVED__01            = 8
uint8           RESERVED__02            = 9
uint8           RESERVED__03            = 10
uint8           RESERVED__04            = 11
uint8           RESERVED__05            = 12
uint8           RESERVED__06            = 13
uint8           RESERVED__07            = 14
uint8           RESERVED__08            = 15

# ********** END of LaneAttributes_Parking   **********
