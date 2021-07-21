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
#    laneID LaneID ,
#    name DescriptiveName OPTIONAL ,
#    ingressApproach ApproachID OPTIONAL,
#    egressApproach ApproachID OPTIONAL ,
#    laneAttributes LaneAttributes ,
#    maneuvers AllowedManeuvers OPTIONAL ,
#    nodeList NodeListXY ,
#    connectsTo ConnectsToList OPTIONAL ,
#    overlays OverlayLaneList OPTIONAL ,
#    ... 
# } 
#
# ********** START of GenericLane **********
#

bool             namePresent
bool             ingressApproachPresent
bool             egressApproachPresent
bool             maneuversPresent
bool             connectsToPresent
bool             overlaysPresent

LaneID           laneID                 #  The unique ID number assigned 
                                        #  to this lane object
DescriptiveName  name                   #  often for debug use only 
                                        #  but at times used to name ped crossings
                                        # OPTIONAL
ApproachID       ingressApproach        # OPTIONAL
ApproachID       egressApproach         #  Approach IDs to which this lane belongs
                                        # OPTIONAL
LaneAttributes   laneAttributes         #  All Attribute information about 
                                        #  the basic selected lane type 
                                        #  Directions of use, Geometric co-sharing 
                                        #  and Type Specific Attributes 
                                        #  These Attributes are 'lane - global' that is,
                                        #  they are true for the entire length of the lane
AllowedManeuvers maneuvers              #  the permitted maneuvers for this lane
                                        # OPTIONAL
NodeListXY       nodeList               #  Lane spatial path information as well as 
                                        #  various Attribute information along the node path  
                                        #  Attributes found here are more general and may 
                                        #  come and go over the length of the lane. 
ConnectsToList   connectsTo             #  a list of other lanes and their signal group IDs 
                                        #  each connecting lane and its signal group ID 
                                        #  is given, therefore this element provides the 
                                        #  information formerly in "signalGroups" in prior 
                                        #  editions. 
                                        # OPTIONAL
OverlayLaneList  overlays               #  A list of any lanes which have spatial paths that
                                        #  overlay (run on top of, and not simply cross) 
                                        #  the path of this lane when used
                                        # OPTIONAL

# ********** END of GenericLane   **********
