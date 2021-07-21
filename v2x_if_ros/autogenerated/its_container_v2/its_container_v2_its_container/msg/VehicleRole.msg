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
# ENUMERATED { default_(0), publicTransport(1), specialTransport(2), dangerousGoods(3), roadWork(4), rescue(5), emergency(6), safetyCar(7), agriculture(8), commercial(9), military(10), roadOperator(11), taxi(12), reserved1(13), reserved2(14), reserved3(15) } 
#
# ********** START of VehicleRole **********
#

int8            value

int8            MIN               = 0
int8            MAX               = 15

int8            DEFAULT__         = 0
int8            PUBLIC_TRANSPORT  = 1
int8            SPECIAL_TRANSPORT = 2
int8            DANGEROUS_GOODS   = 3
int8            ROAD_WORK         = 4
int8            RESCUE            = 5
int8            EMERGENCY         = 6
int8            SAFETY_CAR        = 7
int8            AGRICULTURE       = 8
int8            COMMERCIAL        = 9
int8            MILITARY          = 10
int8            ROAD_OPERATOR     = 11
int8            TAXI              = 12
int8            RESERVED_1        = 13
int8            RESERVED_2        = 14
int8            RESERVED_3        = 15

# ********** END of VehicleRole   **********
