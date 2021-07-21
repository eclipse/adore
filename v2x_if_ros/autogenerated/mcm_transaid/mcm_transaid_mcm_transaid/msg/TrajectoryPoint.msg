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
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    deltaXCm DiffPosition,
#    deltaYCm DiffPosition,
#    deltaTimeMs DiffTime,
#    absSpeed SpeedValue OPTIONAL,
#    longitudinalAcceleration LongitudinalAccelerationValue OPTIONAL 
# } 
#
# ********** START of TrajectoryPoint **********
#

bool                                                         absSpeedPresent
bool                                                         longitudinalAccelerationPresent

DiffPosition                                                 deltaXCm
DiffPosition                                                 deltaYCm
DiffTime                                                     deltaTimeMs
its_container_v1_its_container/SpeedValue                    absSpeed                        # OPTIONAL
its_container_v1_its_container/LongitudinalAccelerationValue longitudinalAcceleration        # 
                                                                                             # Vehicle longitudinal acceleration of the originating ITS-S in the center
                                                                                             # of the mass of the empty vehicle. It shall include a confidence level
                                                                                             # of 95%, otherwise set to unavailable.
                                                                                             # OPTIONAL

# ********** END of TrajectoryPoint   **********
