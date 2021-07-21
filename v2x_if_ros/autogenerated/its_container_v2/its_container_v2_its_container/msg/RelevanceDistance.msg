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
# ENUMERATED { lessThan50m(0), lessThan100m(1), lessThan200m(2), lessThan500m(3), lessThan1000m(4), lessThan5km(5), lessThan10km(6), over10km(7) } 
#
# ********** START of RelevanceDistance **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 7

int8            LESS_THAN_50M   = 0
int8            LESS_THAN_100M  = 1
int8            LESS_THAN_200M  = 2
int8            LESS_THAN_500M  = 3
int8            LESS_THAN_1000M = 4
int8            LESS_THAN_5KM   = 5
int8            LESS_THAN_10KM  = 6
int8            OVER_10KM       = 7

# ********** END of RelevanceDistance   **********
