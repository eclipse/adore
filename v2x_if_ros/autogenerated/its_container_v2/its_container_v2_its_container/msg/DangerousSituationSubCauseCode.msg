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
# INTEGER { unavailable(0), emergencyElectronicBrakeEngaged(1), preCrashSystemEngaged(2), espEngaged(3), absEngaged(4), aebEngaged(5), brakeWarningEngaged(6), collisionRiskWarningEngaged(7) }(0..255) 
#
# ********** START of DangerousSituationSubCauseCode **********
#

uint8           value

uint8           MIN                                = 0
uint8           MAX                                = 255
uint8           UNAVAILABLE                        = 0

uint8           EMERGENCY_ELECTRONIC_BRAKE_ENGAGED = 1
uint8           PRE_CRASH_SYSTEM_ENGAGED           = 2
uint8           ESP_ENGAGED                        = 3
uint8           ABS_ENGAGED                        = 4
uint8           AEB_ENGAGED                        = 5
uint8           BRAKE_WARNING_ENGAGED              = 6
uint8           COLLISION_RISK_WARNING_ENGAGED     = 7

# ********** END of DangerousSituationSubCauseCode   **********
