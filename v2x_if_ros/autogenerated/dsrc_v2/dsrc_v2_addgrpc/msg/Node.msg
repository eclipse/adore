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
#    id INTEGER,
#    lane LaneID OPTIONAL,
#    connectionID LaneConnectionID OPTIONAL,
#    intersectionID IntersectionID OPTIONAL,
#    ... 
# } 
#
# ********** START of Node **********
#

bool                          lanePresent
bool                          connectionIDPresent
bool                          intersectionIDPresent

Node_id                       id
dsrc_v2_dsrc/LaneID           lane                  # OPTIONAL
dsrc_v2_dsrc/LaneConnectionID connectionID          # OPTIONAL
dsrc_v2_dsrc/IntersectionID   intersectionID        # 
                                                    #  The values zero through 255 are allocated for testing purposes 
                                                    #  Note that the value assigned to an intersection will be 
                                                    #  unique within a given regional ID only 
                                                    # OPTIONAL

# ********** END of Node   **********
