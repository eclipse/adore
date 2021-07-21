#* Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)        *
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
#* File automatically generated with DLR Wind (07.10.2020 16:24:31)                 *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v1:1.2     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(1)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER { reserved(0), trafficCondition(1), accident(2), roadworks(3), adverseWeatherCondition_Adhesion(6), hazardousLocation_SurfaceCondition(9), hazardousLocation_ObstacleOnTheRoad(10), hazardousLocation_AnimalOnTheRoad(11), humanPresenceOnTheRoad(12), wrongWayDriving(14), rescueAndRecoveryWorkInProgress(15), adverseWeatherCondition_ExtremeWeatherCondition(17), adverseWeatherCondition_Visibility(18), adverseWeatherCondition_Precipitation(19), slowVehicle(26), dangerousEndOfQueue(27), vehicleBreakdown(91), postCrash(92), humanProblem(93), stationaryVehicle(94), emergencyVehicleApproaching(95), hazardousLocation_DangerousCurve(96), collisionRisk(97), signalViolation(98), dangerousSituation(99) }(0..255) 
#
# ********** START of CauseCodeType **********
#

uint8           value

uint8           MIN             = 0
uint8           MAX             = 255

# ********** END of CauseCodeType   **********
