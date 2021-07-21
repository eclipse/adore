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
# BIT STRING { maneuverStraightAllowed(0), maneuverLeftAllowed(1), maneuverRightAllowed(2), maneuverUTurnAllowed(3), maneuverLeftTurnOnRedAllowed(4), maneuverRightTurnOnRedAllowed(5), maneuverLaneChangeAllowed(6), maneuverNoStoppingAllowed(7), yieldAllwaysRequired(8), goWithHalt(9), caution(10), reserved1(11) }(SIZE(12)) 
#
# ********** START of AllowedManeuvers **********
#

bool[]          values

uint8           ARRAY_SIZE                         = 12

uint8           MANEUVER_STRAIGHT_ALLOWED          = 0
uint8           MANEUVER_LEFT_ALLOWED              = 1
uint8           MANEUVER_RIGHT_ALLOWED             = 2
uint8           MANEUVER_U_TURN_ALLOWED            = 3
uint8           MANEUVER_LEFT_TURN_ON_RED_ALLOWED  = 4
uint8           MANEUVER_RIGHT_TURN_ON_RED_ALLOWED = 5
uint8           MANEUVER_LANE_CHANGE_ALLOWED       = 6
uint8           MANEUVER_NO_STOPPING_ALLOWED       = 7
uint8           YIELD_ALLWAYS_REQUIRED             = 8
uint8           GO_WITH_HALT                       = 9
uint8           CAUTION                            = 10
uint8           RESERVED_1                         = 11

# ********** END of AllowedManeuvers   **********
