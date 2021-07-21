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
#    localNode NodeAttributeXYList OPTIONAL ,
#    disabled SegmentAttributeXYList OPTIONAL ,
#    enabled SegmentAttributeXYList OPTIONAL ,
#    data LaneDataAttributeList OPTIONAL ,
#    dWidth Offset_B10 OPTIONAL ,
#    dElevation Offset_B10 OPTIONAL ,
#    ... 
# } 
#
# ********** START of NodeAttributeSetXY **********
#

bool                   localNodePresent
bool                   disabledPresent
bool                   enabledPresent
bool                   dataPresent
bool                   dWidthPresent
bool                   dElevationPresent

NodeAttributeXYList    localNode         #  Attribute states which pertain to this node point
                                         # OPTIONAL
SegmentAttributeXYList disabled          #  Attribute states which are disabled at this node point
                                         # OPTIONAL
SegmentAttributeXYList enabled           #  Attribute states which are enabled at this node point
                                         #  and which remain enabled until disabled or the lane ends
                                         # OPTIONAL
LaneDataAttributeList  data              #  Attributes which require an additional data values
                                         #  some of these are local to the node point, while others
                                         #  persist with the provided values until changed
                                         #  and this is indicated in each entry
                                         # OPTIONAL
Offset_B10             dWidth            #  A value added to the current lane width
                                         #  at this node and from this node onwards, in 1cm steps
                                         #  lane width between nodes are a linear taper between pts
                                         #  the value of zero shall not be sent here
                                         # 
                                         #  a range of +- 5.11 meters 
                                         # OPTIONAL
Offset_B10             dElevation        #  A value added to the current Elevation 
                                         #  at this node from this node onwards, in 10cm steps
                                         #  elevations between nodes are a linear taper between pts
                                         #  the value of zero shall not be sent here
                                         # 
                                         #  a range of +- 5.11 meters 
                                         # OPTIONAL

# ********** END of NodeAttributeSetXY   **********
