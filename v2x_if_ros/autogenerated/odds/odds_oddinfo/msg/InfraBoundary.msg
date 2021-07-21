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
# BIT STRING { boundaryMark(0), boundaryBarrier(1), boundaryCurb(2), boundaryPole(3) }(SIZE(4)) 
#
# ********** START of InfraBoundary **********
#

bool[]          values

uint8           ARRAY_SIZE       = 4

uint8           BOUNDARY_MARK    = 0
uint8           BOUNDARY_BARRIER = 1
uint8           BOUNDARY_CURB    = 2
uint8           BOUNDARY_POLE    = 3

# ********** END of InfraBoundary   **********
