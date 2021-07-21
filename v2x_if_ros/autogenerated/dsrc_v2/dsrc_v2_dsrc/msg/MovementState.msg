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
#    movementName DescriptiveName OPTIONAL ,
#    signalGroup SignalGroupID ,
#    state_time_speed MovementEventList ,
#    maneuverAssistList ManeuverAssistList OPTIONAL ,
#    ... 
# } 
#
# ********** START of MovementState **********
#

bool               movementNamePresent
bool               maneuverAssistListPresent

DescriptiveName    movementName              #  uniquely defines movement by name   
                                             #  human readable name for intersection  
                                             #  to be used only in debug mode 
                                             # OPTIONAL
SignalGroupID      signalGroup               #  the group id is used to map to lists 
                                             #  of lanes (and their descriptions) 
                                             #  which this MovementState data applies to 
                                             #  see comments in the Remarks for usage details 
MovementEventList  state_time_speed          #  Consisting of sets of movement data with:
                                             #  a)  SignalPhaseState
                                             #  b)  TimeChangeDetails,  and 
                                             #  c)  AdvisorySpeeds      (optional)
                                             #  Note one or more of the movement events may be for 
                                             #  a future time and that this allows conveying multiple 
                                             #  predictive phase and movement timing for various uses
                                             #  for the current signal group
ManeuverAssistList maneuverAssistList        #  This information may also be placed in the 
                                             #  IntersectionState when common information applies to
                                             #  different lanes in the same way
                                             # OPTIONAL

# ********** END of MovementState   **********
