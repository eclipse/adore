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
# ENUMERATED { lessThan1Minute(0), lessThan2Minutes(1), lessThan15Minutes(2), equalOrGreater15Minutes(3) } 
#
# ********** START of StationarySince **********
#

int8            value

int8            MIN                         = 0
int8            MAX                         = 3

int8            LESS_THAN_1_MINUTE          = 0
int8            LESS_THAN_2_MINUTES         = 1
int8            LESS_THAN_15_MINUTES        = 2
int8            EQUAL_OR_GREATER_15_MINUTES = 3

# ********** END of StationarySince   **********
