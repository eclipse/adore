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
# INTEGER { unavailable(0), majorRoadworks(1), roadMarkingWork(2), slowMovingRoadMaintenance(3), shortTermStationaryRoadworks(4), streetCleaning(5), winterService(6) }(0..255) 
#
# ********** START of RoadworksSubCauseCode **********
#

uint8           value

uint8           MIN                             = 0
uint8           MAX                             = 255
uint8           UNAVAILABLE                     = 0

uint8           MAJOR_ROADWORKS                 = 1
uint8           ROAD_MARKING_WORK               = 2
uint8           SLOW_MOVING_ROAD_MAINTENANCE    = 3
uint8           SHORT_TERM_STATIONARY_ROADWORKS = 4
uint8           STREET_CLEANING                 = 5
uint8           WINTER_SERVICE                  = 6

# ********** END of RoadworksSubCauseCode   **********
