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
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    referenceLaneId LaneID,
#    offsetXaxis CHOICE {
#    small DrivenLineOffsetSm,
#    large DrivenLineOffsetLg 
# },
#    offsetYaxis CHOICE {
#    small DrivenLineOffsetSm,
#    large DrivenLineOffsetLg 
# },
#    rotateXY Angle OPTIONAL,
#    scaleXaxis Scale_B12 OPTIONAL,
#    scaleYaxis Scale_B12 OPTIONAL,
#    ... 
# } 
#
# ********** START of ComputedLane **********
#

bool                     rotateXYPresent
bool                     scaleXaxisPresent
bool                     scaleYaxisPresent

LaneID                   referenceLaneId
ComputedLane_offsetXaxis offsetXaxis
ComputedLane_offsetYaxis offsetYaxis
Angle                    rotateXY          # OPTIONAL
Scale_B12                scaleXaxis        # 
                                           # in steps of 0.05 percent
                                           # OPTIONAL
Scale_B12                scaleYaxis        # 
                                           # in steps of 0.05 percent
                                           # OPTIONAL

# ********** END of ComputedLane   **********
