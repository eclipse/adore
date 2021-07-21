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
# INTEGER { unavailable(0), shedLoad(1), partsOfVehicles(2), partsOfTyres(3), bigObjects(4), fallenTrees(5), hubCaps(6), waitingVehicles(7) }(0..255) 
#
# ********** START of HazardousLocation_ObstacleOnTheRoadSubCauseCode **********
#

uint8           value

uint8           MIN               = 0
uint8           MAX               = 255
uint8           UNAVAILABLE       = 0

uint8           SHED_LOAD         = 1
uint8           PARTS_OF_VEHICLES = 2
uint8           PARTS_OF_TYRES    = 3
uint8           BIG_OBJECTS       = 4
uint8           FALLEN_TREES      = 5
uint8           HUB_CAPS          = 6
uint8           WAITING_VEHICLES  = 7

# ********** END of HazardousLocation_ObstacleOnTheRoadSubCauseCode   **********
