#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                             *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                                         *
#*                                                                                                       *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                              *
#*                                                                                                       *
#* This program and the accompanying materials are made available under the                              *
#* terms of the Eclipse Public License 2.0 which is available at                                         *
#* http://www.eclipse.org/legal/epl-2.0.                                                                 *
#*                                                                                                       *
#* SPDX-License-Identifier: EPL-2.0                                                                      *
#*                                                                                                       *
#*                                                                                                       *
#* File automatically generated with DLR Wind v2 (2021)                                                  *
#*                                                                                                       *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3                                 *
#*                                                                                                       *
#* Module: CPM_SensorInformation_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(103324) sensorInformation(4) version1(1)}                                               *
#*                                                                                                       *
#* For support contact v2x-ts@dlr.de                                                                     *
#*                                                                                                       *

#
# Element type: CHOICE
# 
# ASN.1 Definition (Copyright 2019 ETSI):
# CHOICE {
#    node_Z1 Offset_B10,
#    node_Z2 Offset_B11,
#    node_Z3 Offset_B12,
#    node_Z4 Offset_B13,
#    node_Z5 Offset_B14,
#    node_Z6 Offset_B16 
# } 
#
# ********** START of NodeOffsetPointZ **********
#

uint8                   choice          # Use this field to indicate which of the following fields contains valid data.

dsrc_v2_dsrc/Offset_B10 node_Z1         # choice = 0
dsrc_v2_dsrc/Offset_B11 node_Z2         # choice = 1
dsrc_v2_dsrc/Offset_B12 node_Z3         # choice = 2
dsrc_v2_dsrc/Offset_B13 node_Z4         # choice = 3
dsrc_v2_dsrc/Offset_B14 node_Z5         # choice = 4
dsrc_v2_dsrc/Offset_B16 node_Z6         # choice = 5

# ********** END of NodeOffsetPointZ   **********
