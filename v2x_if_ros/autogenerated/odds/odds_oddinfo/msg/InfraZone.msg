#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de) *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)             *
#*                                                                           *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore  *
#*                                                                           *
#* This program and the accompanying materials are made available under the  *
#* terms of the Eclipse Public License 2.0 which is available at             *
#* http://www.eclipse.org/legal/epl-2.0.                                     *
#*                                                                           *
#* SPDX-License-Identifier: EPL-2.0                                          *
#*                                                                           *
#*                                                                           *
#* File automatically generated with DLR Wind v2 (2021)                      *
#*                                                                           *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0         *
#*                                                                           *
#* Module: ODDInfo {}                                                        *
#*                                                                           *
#* For support contact v2x-ts@dlr.de                                         *
#*                                                                           *

#
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { tunnel(0), parkingDeck(1), parkingArea(2), undergroundParking(3), city(4), bridge(5), school(6), playZone(7) }(SIZE(8)) 
#
# ********** START of InfraZone **********
#

bool[]          values

uint8           ARRAY_SIZE          = 8

uint8           TUNNEL              = 0
uint8           PARKING_DECK        = 1
uint8           PARKING_AREA        = 2
uint8           UNDERGROUND_PARKING = 3
uint8           CITY                = 4
uint8           BRIDGE              = 5
uint8           SCHOOL              = 6
uint8           PLAY_ZONE           = 7

# ********** END of InfraZone   **********
