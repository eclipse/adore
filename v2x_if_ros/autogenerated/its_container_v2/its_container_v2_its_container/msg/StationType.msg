#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)        *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                    *
#*                                                                                  *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore         *
#*                                                                                  *
#* This program and the accompanying materials are made available under the         *
#* terms of the Eclipse Public License 2.0 which is available at                    *
#* http://www.eclipse.org/legal/epl-2.0.                                            *
#*                                                                                  *
#* SPDX-License-Identifier: EPL-2.0                                                 *
#*                                                                                  *
#*                                                                                  *
#* File automatically generated with DLR Wind v2 (2021)                             *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.4     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(2)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER { unknown(0), pedestrian(1), cyclist(2), moped(3), motorcycle(4), passengerCar(5), bus(6), lightTruck(7), heavyTruck(8), trailer(9), specialVehicles(10), tram(11), roadSideUnit(15) }(0..255) 
#
# ********** START of StationType **********
#

uint8           value

uint8           MIN              = 0
uint8           MAX              = 255

uint8           UNKNOWN          = 0
uint8           PEDESTRIAN       = 1
uint8           CYCLIST          = 2
uint8           MOPED            = 3
uint8           MOTORCYCLE       = 4
uint8           PASSENGER_CAR    = 5
uint8           BUS              = 6
uint8           LIGHT_TRUCK      = 7
uint8           HEAVY_TRUCK      = 8
uint8           TRAILER          = 9
uint8           SPECIAL_VEHICLES = 10
uint8           TRAM             = 11
uint8           ROAD_SIDE_UNIT   = 15

# ********** END of StationType   **********
