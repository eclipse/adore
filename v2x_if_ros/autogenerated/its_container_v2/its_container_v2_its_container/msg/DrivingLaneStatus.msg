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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { value(0) }(SIZE(1..13)) 
#
# ********** START of DrivingLaneStatus **********
#

bool[]          values

uint8           ARRAY_SIZE      = 13

uint8           VALUE           = 0
uint8           RESERVED__00    = 1
uint8           RESERVED__01    = 2
uint8           RESERVED__02    = 3
uint8           RESERVED__03    = 4
uint8           RESERVED__04    = 5
uint8           RESERVED__05    = 6
uint8           RESERVED__06    = 7
uint8           RESERVED__07    = 8
uint8           RESERVED__08    = 9
uint8           RESERVED__09    = 10
uint8           RESERVED__10    = 11
uint8           RESERVED__11    = 12

# ********** END of DrivingLaneStatus   **********
