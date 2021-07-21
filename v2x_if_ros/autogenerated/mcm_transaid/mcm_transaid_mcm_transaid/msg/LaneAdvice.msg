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
#    adviceID AdviceID,
#    laneAdviceReason LaneAdviceReason,
#    laneChangePosition LaneChangePosition,
#    laneChangeMoment LaneChangeMoment,
#    laneChangeSpeed LaneChangeSpeed OPTIONAL,
#    leadingVehicle LeadingVehicle OPTIONAL,
#    followingVehicle FollowingVehicle OPTIONAL,
#    targetLane TargetLane,
#    triggeringPointOfToC TriggeringPointOfToC OPTIONAL 
# } 
#
# ********** START of LaneAdvice **********
#

bool                 laneChangeSpeedPresent
bool                 leadingVehiclePresent
bool                 followingVehiclePresent
bool                 triggeringPointOfToCPresent

AdviceID             adviceID
LaneAdviceReason     laneAdviceReason
LaneChangePosition   laneChangePosition
LaneChangeMoment     laneChangeMoment
LaneChangeSpeed      laneChangeSpeed             # OPTIONAL
LeadingVehicle       leadingVehicle              # OPTIONAL
FollowingVehicle     followingVehicle            # OPTIONAL
TargetLane           targetLane
TriggeringPointOfToC triggeringPointOfToC        # OPTIONAL

# ********** END of LaneAdvice   **********
