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
#    name DescriptiveName OPTIONAL,
#    id IntersectionReferenceID,
#    revision MsgCount,
#    status IntersectionStatusObject,
#    moy MinuteOfTheYear OPTIONAL,
#    timeStamp DSecond OPTIONAL,
#    enabledLanes EnabledLaneList OPTIONAL,
#    states MovementList,
#    maneuverAssistList ManeuverAssistList OPTIONAL,
#    ... 
# } 
#
# ********** START of IntersectionState **********
#

bool                     namePresent
bool                     moyPresent
bool                     timeStampPresent
bool                     enabledLanesPresent
bool                     maneuverAssistListPresent

DescriptiveName          name                      # OPTIONAL
IntersectionReferenceID  id
MsgCount                 revision
IntersectionStatusObject status
MinuteOfTheYear          moy                       # 
                                                   #  the value 527040 shall be used for invalid 
                                                   # OPTIONAL
DSecond                  timeStamp                 # OPTIONAL
EnabledLaneList          enabledLanes              # 
                                                   #  The unique ID numbers for each
                                                   #  lane object which is 'active' 
                                                   #  as part of the dynamic map contents.
                                                   # OPTIONAL
MovementList             states
ManeuverAssistList       maneuverAssistList        # OPTIONAL

# ********** END of IntersectionState   **********
