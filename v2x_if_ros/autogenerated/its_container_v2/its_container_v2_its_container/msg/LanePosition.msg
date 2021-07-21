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
# INTEGER { offTheRoad(_1), innerHardShoulder(0), innermostDrivingLane(1), secondLaneFromInside(2), outerHardShoulder(14) }(-1..14) 
#
# ********** START of LanePosition **********
#

int8            value

int8            MIN                     = -1
int8            MAX                     = 14

int8            OFF_THE_ROAD            = -1
int8            INNER_HARD_SHOULDER     = 0
int8            INNERMOST_DRIVING_LANE  = 1
int8            SECOND_LANE_FROM_INSIDE = 2
int8            OUTER_HARD_SHOULDER     = 14

# ********** END of LanePosition   **********
