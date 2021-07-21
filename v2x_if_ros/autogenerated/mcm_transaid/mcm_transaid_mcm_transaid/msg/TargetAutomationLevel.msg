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
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_transaid:3.0  *
#*                                                                           *
#* Module: MCM_TransAID {version(3)}                                         *
#*                                                                           *
#* For support contact v2x-ts@dlr.de                                         *
#*                                                                           *

#
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { saeLevel0(0), saeLevel1(1), saeLevel2(2), saeLevel3(3), saeLevel4(4), saeLevel5(5), ... } 
#
# ********** START of TargetAutomationLevel **********
#

int8            value

int8            MIN             = 0
int8            MAX             = 5

int8            SAE_LEVEL_0     = 0
int8            SAE_LEVEL_1     = 1
int8            SAE_LEVEL_2     = 2
int8            SAE_LEVEL_3     = 3
int8            SAE_LEVEL_4     = 4
int8            SAE_LEVEL_5     = 5

# ********** END of TargetAutomationLevel   **********
