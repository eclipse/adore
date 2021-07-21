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
#    connectingLane ConnectingLane ,
#    remoteIntersection IntersectionReferenceID OPTIONAL ,
#    signalGroup SignalGroupID OPTIONAL ,
#    userClass RestrictionClassID OPTIONAL ,
#    connectionID LaneConnectionID OPTIONAL  
# } 
#
# ********** START of Connection **********
#

bool                    remoteIntersectionPresent
bool                    signalGroupPresent
bool                    userClassPresent
bool                    connectionIDPresent

ConnectingLane          connectingLane            #  The index of the connecting lane and also
                                                  #  the maneuver from the current lane to it
IntersectionReferenceID remoteIntersection        #  This entry is only used when the 
                                                  #  indicated connecting lane belongs 
                                                  #  to another intersection layout. This 
                                                  #  provides a means to create meshes of lanes 
                                                  # OPTIONAL
SignalGroupID           signalGroup               #  The matching signal group send by 
                                                  #  the SPAT message for this lane/maneuver.
                                                  #  Shall be present unless the connectingLane 
                                                  #  has no signal group (is un-signalized) 
                                                  # OPTIONAL
RestrictionClassID      userClass                 #  The Restriction Class of users this applies to 
                                                  #  The use of some lane/maneuver and SignalGroupID 
                                                  #  pairings are restricted to selected users. 
                                                  #  When absent, the SignalGroupID applies to all 
                                                  # OPTIONAL
LaneConnectionID        connectionID              #  An optional connection index used to 
                                                  #  relate this lane connection to any dynamic 
                                                  #  clearance data in the SPAT. Note that
                                                  #  the index may be shared with other 
                                                  #  connections if the clearance data is common
                                                  # OPTIONAL

# ********** END of Connection   **********
