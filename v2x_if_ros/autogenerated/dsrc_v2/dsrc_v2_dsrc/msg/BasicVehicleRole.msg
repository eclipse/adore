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
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { basicVehicle(0), publicTransport(1), specialTransport(2), dangerousGoods(3), roadWork(4), roadRescue(5), emergency(6), safetyCar(7), none_unknown(8), truck(9), motorcycle(10), roadSideSource(11), police(12), fire(13), ambulance(14), dot(15), transit(16), slowMoving(17), stopNgo(18), cyclist(19), pedestrian(20), nonMotorized(21), military(22), ... } 
#
# ********** START of BasicVehicleRole **********
#

int8            value

int8            MIN               = 0
int8            MAX               = 22

int8            BASIC_VEHICLE     = 0
int8            PUBLIC_TRANSPORT  = 1
int8            SPECIAL_TRANSPORT = 2
int8            DANGEROUS_GOODS   = 3
int8            ROAD_WORK         = 4
int8            ROAD_RESCUE       = 5
int8            EMERGENCY         = 6
int8            SAFETY_CAR        = 7
int8            NONE__UNKNOWN     = 8
int8            TRUCK             = 9
int8            MOTORCYCLE        = 10
int8            ROAD_SIDE_SOURCE  = 11
int8            POLICE            = 12
int8            FIRE              = 13
int8            AMBULANCE         = 14
int8            DOT               = 15
int8            TRANSIT           = 16
int8            SLOW_MOVING       = 17
int8            STOP_NGO          = 18
int8            CYCLIST           = 19
int8            PEDESTRIAN        = 20
int8            NON_MOTORIZED     = 21
int8            MILITARY          = 22

# ********** END of BasicVehicleRole   **********
