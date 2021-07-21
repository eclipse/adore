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
# INTEGER { unavailable(0), heavyFrostOnRoad(1), fuelOnRoad(2), mudOnRoad(3), snowOnRoad(4), iceOnRoad(5), blackIceOnRoad(6), oilOnRoad(7), looseChippings(8), instantBlackIce(9), roadsSalted(10) }(0..255) 
#
# ********** START of AdverseWeatherCondition_AdhesionSubCauseCode **********
#

uint8           value

uint8           MIN                 = 0
uint8           MAX                 = 255
uint8           UNAVAILABLE         = 0

uint8           HEAVY_FROST_ON_ROAD = 1
uint8           FUEL_ON_ROAD        = 2
uint8           MUD_ON_ROAD         = 3
uint8           SNOW_ON_ROAD        = 4
uint8           ICE_ON_ROAD         = 5
uint8           BLACK_ICE_ON_ROAD   = 6
uint8           OIL_ON_ROAD         = 7
uint8           LOOSE_CHIPPINGS     = 8
uint8           INSTANT_BLACK_ICE   = 9
uint8           ROADS_SALTED        = 10

# ********** END of AdverseWeatherCondition_AdhesionSubCauseCode   **********
