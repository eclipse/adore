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
int64 iteration             # the planning iteration
float64 t_planning_start    # plannning start time
float64 t_planning_end      # terminate planning for given iteration at this time
float64 t_emergency_start   # time at which nominal maneuver transitions to emergency maneuver
SetPoint initial_state      # where the plan should start