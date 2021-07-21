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
#    name DescriptiveName OPTIONAL ,
#    id IntersectionReferenceID ,
#    revision MsgCount,
#    refPoint Position3D,
#    laneWidth LaneWidth OPTIONAL ,
#    speedLimits SpeedLimitList OPTIONAL ,
#    laneSet LaneList,
#    ... 
# } 
#
# ********** START of IntersectionGeometry **********
#

bool                    namePresent
bool                    laneWidthPresent
bool                    speedLimitsPresent

DescriptiveName         name               #  For debug use only 
                                           # OPTIONAL
IntersectionReferenceID id                 #  A globally unique value set, 
                                           #  consisting of a regionID and 
                                           #  intersection ID assignment
MsgCount                revision
Position3D              refPoint
LaneWidth               laneWidth          #  Reference width used by all subsequent 
                                           #  lanes unless a new width is given 
                                           # 
                                           #  units of 1 cm
                                           # OPTIONAL
SpeedLimitList          speedLimits        #  Reference regulatory speed limits
                                           #  used by all subsequent 
                                           #  lanes unless a new speed is given 
                                           #  See Section 11 for converting and 
                                           #  translating speed expressed in mph 
                                           #  into units of m/s 
                                           # OPTIONAL
LaneList                laneSet

# ********** END of IntersectionGeometry   **********
