#* Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)        *
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
#* File automatically generated with DLR Wind (07.10.2020 16:24:31)                 *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v1:1.2     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(1)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { row1LeftOccupied(0), row1RightOccupied(1), row1MidOccupied(2), row1NotDetectable(3), row1NotPresent(4), row2LeftOccupied(5), row2RightOccupied(6), row2MidOccupied(7), row2NotDetectable(8), row2NotPresent(9), row3LeftOccupied(10), row3RightOccupied(11), row3MidOccupied(12), row3NotDetectable(13), row3NotPresent(14), row4LeftOccupied(15), row4RightOccupied(16), row4MidOccupied(17), row4NotDetectable(18), row4NotPresent(19) }(SIZE(20)) 
#
# ********** START of PositionOfOccupants **********
#

bool[]          values

uint8           ARRAY_SIZE      = 20

# ********** END of PositionOfOccupants   **********
