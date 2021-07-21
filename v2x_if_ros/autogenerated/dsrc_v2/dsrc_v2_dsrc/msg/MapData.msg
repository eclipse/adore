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
#    timeStamp MinuteOfTheYear OPTIONAL,
#    msgIssueRevision MsgCount,
#    layerType LayerType OPTIONAL,
#    layerID LayerID OPTIONAL,
#    intersections IntersectionGeometryList OPTIONAL,
#    restrictionList RestrictionClassList OPTIONAL,
#    ... 
# } 
#
# ********** START of MapData **********
#

bool                     timeStampPresent
bool                     layerTypePresent
bool                     layerIDPresent
bool                     intersectionsPresent
bool                     restrictionListPresent

MinuteOfTheYear          timeStamp              # 
                                                #  the value 527040 shall be used for invalid 
                                                # OPTIONAL
MsgCount                 msgIssueRevision
LayerType                layerType              # OPTIONAL
LayerID                  layerID                # OPTIONAL
IntersectionGeometryList intersections          # OPTIONAL
RestrictionClassList     restrictionList        # OPTIONAL

# ********** END of MapData   **********
