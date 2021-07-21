#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)        *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                    *
#*                                                                                  *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore         *
#*                                                                                  *
#* This program and the accompanying materials are made available under the         *
#* terms of the Eclipse Public License 2.0 which is available at                    *
#* http://www.eclipse.org/legal/epl-2.0.                                            *
#*                                                                                  *
#* SPDX-License-Identifier: EPL-2.0                                                 *
#*                                                                                  *
#*                                                                                  *
#* File automatically generated with DLR Wind v2 (2021)                             *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.4     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(2)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { urban_NoStructuralSeparationToOppositeLanes(0), urban_WithStructuralSeparationToOppositeLanes(1), nonUrban_NoStructuralSeparationToOppositeLanes(2), nonUrban_WithStructuralSeparationToOppositeLanes(3) } 
#
# ********** START of RoadType **********
#

int8            value

int8            MIN                                                      = 0
int8            MAX                                                      = 3

int8            URBAN___NO_STRUCTURAL_SEPARATION_TO_OPPOSITE_LANES       = 0
int8            URBAN___WITH_STRUCTURAL_SEPARATION_TO_OPPOSITE_LANES     = 1
int8            NON_URBAN___NO_STRUCTURAL_SEPARATION_TO_OPPOSITE_LANES   = 2
int8            NON_URBAN___WITH_STRUCTURAL_SEPARATION_TO_OPPOSITE_LANES = 3

# ********** END of RoadType   **********
