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
# BIT STRING { lowBeamHeadlightsOn(0), highBeamHeadlightsOn(1), leftTurnSignalOn(2), rightTurnSignalOn(3), daytimeRunningLightsOn(4), reverseLightOn(5), fogLightOn(6), parkingLightsOn(7) }(SIZE(8)) 
#
# ********** START of ExteriorLights **********
#

bool[]          values

uint8           ARRAY_SIZE                = 8

uint8           LOW_BEAM_HEADLIGHTS_ON    = 0
uint8           HIGH_BEAM_HEADLIGHTS_ON   = 1
uint8           LEFT_TURN_SIGNAL_ON       = 2
uint8           RIGHT_TURN_SIGNAL_ON      = 3
uint8           DAYTIME_RUNNING_LIGHTS_ON = 4
uint8           REVERSE_LIGHT_ON          = 5
uint8           FOG_LIGHT_ON              = 6
uint8           PARKING_LIGHTS_ON         = 7

# ********** END of ExteriorLights   **********
