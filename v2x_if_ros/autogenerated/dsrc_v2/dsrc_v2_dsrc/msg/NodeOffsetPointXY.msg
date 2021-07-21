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
# Element type: CHOICE
# 
# ASN.1 Definition:
# CHOICE {
#    node_XY1 Node_XY_20b,
#    node_XY2 Node_XY_22b,
#    node_XY3 Node_XY_24b,
#    node_XY4 Node_XY_26b,
#    node_XY5 Node_XY_28b,
#    node_XY6 Node_XY_32b,
#    node_LatLon Node_LLmD_64b 
# } 
#
# ********** START of NodeOffsetPointXY **********
#

uint8           choice          # Use this field to indicate which of the following fields contains valid data.

Node_XY_20b     node_XY1        # choice = 0
Node_XY_22b     node_XY2        # choice = 1
Node_XY_24b     node_XY3        # choice = 2
Node_XY_26b     node_XY4        # choice = 3
Node_XY_28b     node_XY5        # choice = 4
Node_XY_32b     node_XY6        # choice = 5
Node_LLmD_64b   node_LatLon     # choice = 6

# ********** END of NodeOffsetPointXY   **********
