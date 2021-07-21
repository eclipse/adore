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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { brakePedalEngaged(0), gasPedalEngaged(1), emergencyBrakeEngaged(2), collisionWarningEngaged(3), accEngaged(4), cruiseControlEngaged(5), speedLimiterEngaged(6) }(SIZE(7)) 
#
# ********** START of AccelerationControl **********
#

bool[]          values

uint8           ARRAY_SIZE                = 7

uint8           BRAKE_PEDAL_ENGAGED       = 0
uint8           GAS_PEDAL_ENGAGED         = 1
uint8           EMERGENCY_BRAKE_ENGAGED   = 2
uint8           COLLISION_WARNING_ENGAGED = 3
uint8           ACC_ENGAGED               = 4
uint8           CRUISE_CONTROL_ENGAGED    = 5
uint8           SPEED_LIMITER_ENGAGED     = 6

# ********** END of AccelerationControl   **********
