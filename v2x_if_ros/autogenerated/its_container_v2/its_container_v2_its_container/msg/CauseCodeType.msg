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
# INTEGER { reserved(0), trafficCondition(1), accident(2), roadworks(3), impassability(5), adverseWeatherCondition_Adhesion(6), aquaplannning(7), hazardousLocation_SurfaceCondition(9), hazardousLocation_ObstacleOnTheRoad(10), hazardousLocation_AnimalOnTheRoad(11), humanPresenceOnTheRoad(12), wrongWayDriving(14), rescueAndRecoveryWorkInProgress(15), adverseWeatherCondition_ExtremeWeatherCondition(17), adverseWeatherCondition_Visibility(18), adverseWeatherCondition_Precipitation(19), slowVehicle(26), dangerousEndOfQueue(27), vehicleBreakdown(91), postCrash(92), humanProblem(93), stationaryVehicle(94), emergencyVehicleApproaching(95), hazardousLocation_DangerousCurve(96), collisionRisk(97), signalViolation(98), dangerousSituation(99) }(0..255) 
#
# ********** START of CauseCodeType **********
#

uint8           value

uint8           MIN                                                   = 0
uint8           MAX                                                   = 255

uint8           RESERVED                                              = 0
uint8           TRAFFIC_CONDITION                                     = 1
uint8           ACCIDENT                                              = 2
uint8           ROADWORKS                                             = 3
uint8           IMPASSABILITY                                         = 5
uint8           ADVERSE_WEATHER_CONDITION___ADHESION                  = 6
uint8           AQUAPLANNNING                                         = 7
uint8           HAZARDOUS_LOCATION___SURFACE_CONDITION                = 9
uint8           HAZARDOUS_LOCATION___OBSTACLE_ON_THE_ROAD             = 10
uint8           HAZARDOUS_LOCATION___ANIMAL_ON_THE_ROAD               = 11
uint8           HUMAN_PRESENCE_ON_THE_ROAD                            = 12
uint8           WRONG_WAY_DRIVING                                     = 14
uint8           RESCUE_AND_RECOVERY_WORK_IN_PROGRESS                  = 15
uint8           ADVERSE_WEATHER_CONDITION___EXTREME_WEATHER_CONDITION = 17
uint8           ADVERSE_WEATHER_CONDITION___VISIBILITY                = 18
uint8           ADVERSE_WEATHER_CONDITION___PRECIPITATION             = 19
uint8           SLOW_VEHICLE                                          = 26
uint8           DANGEROUS_END_OF_QUEUE                                = 27
uint8           VEHICLE_BREAKDOWN                                     = 91
uint8           POST_CRASH                                            = 92
uint8           HUMAN_PROBLEM                                         = 93
uint8           STATIONARY_VEHICLE                                    = 94
uint8           EMERGENCY_VEHICLE_APPROACHING                         = 95
uint8           HAZARDOUS_LOCATION___DANGEROUS_CURVE                  = 96
uint8           COLLISION_RISK                                        = 97
uint8           SIGNAL_VIOLATION                                      = 98
uint8           DANGEROUS_SITUATION                                   = 99

# ********** END of CauseCodeType   **********
