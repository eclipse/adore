# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Daniel Heß
# ********************************************************************************
int16               id                          # numerical planner id
int64               iteration                   # determines to which planning iteration the planning result contributes
string              name                        # human-readable identifier for or type of maneuver, e.g. "lane following"
SetPointRequest     nominal_maneuver            # the normal maneuver optimzed towards horizon: not to be executed but rather for HMI and decision making
SetPointRequest     combined_maneuver           # combined maneuver starting with nominal_maneuver and transitioning to emergency_maneuver ending in safe state
TerminalRequest     terminal_maneuver           # full-break/standstill after execution of combined maneuver
OccupancyCylinderPrediction nominal_maneuver_swath # space-time volume expected to be covered by nominal_maneuver
OccupancyCylinderPrediction combined_maneuver_swath # space-time volume expected to be covered by combined_maneuver
bool                nominal_maneuver_valid      
bool                combined_maneuver_valid
string              status_string               # string explaining for example why constraint evaluation failed, e.g. "collision with prediction of object 17, branch 3, at t=0.58" or "I/O lin inversion fail"
CostPair[]          objective_values            # set of individual objective values, to be weighted by decision making. e.g. acceleration, jerk, time-loss/navigation-cost
CostPair[]          performance_values          # cpu-time measurements, etc. 
int8                maneuver_type
int8                COLLISION_MITIGATION = 0
int8                EMERGENCY_BREAKING = 1
int8                NOMINAL_DRIVING = 2
bool                indicator_left
bool                indicator_right