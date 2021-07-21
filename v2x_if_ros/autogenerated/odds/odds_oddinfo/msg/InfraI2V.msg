#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de) *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)             *
#*                                                                           *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore  *
#*                                                                           *
#* This program and the accompanying materials are made available under the  *
#* terms of the Eclipse Public License 2.0 which is available at             *
#* http://www.eclipse.org/legal/epl-2.0.                                     *
#*                                                                           *
#* SPDX-License-Identifier: EPL-2.0                                          *
#*                                                                           *
#*                                                                           *
#* File automatically generated with DLR Wind v2 (2021)                      *
#*                                                                           *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0         *
#*                                                                           *
#* Module: ODDInfo {}                                                        *
#*                                                                           *
#* For support contact v2x-ts@dlr.de                                         *
#*                                                                           *

#
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { spatMap(0), vss(1), hazard(2), constructionArea(3), cpm(4) }(SIZE(5)) 
#
# ********** START of InfraI2V **********
#

bool[]          values

uint8           ARRAY_SIZE        = 5

uint8           SPAT_MAP          = 0
uint8           VSS               = 1
uint8           HAZARD            = 2
uint8           CONSTRUCTION_AREA = 3
uint8           CPM               = 4

# ********** END of InfraI2V   **********
