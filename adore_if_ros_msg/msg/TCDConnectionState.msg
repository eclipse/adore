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
#defines state of a connection as controlled by TCD
float64 minEndTime #minimum time at which this state becomes invalid and the next state becomes valid
float64 maxEndTime #maximum time at which this state becomes invalid: optional field
float64 likelyTime #likely time at which this state becomes invalid: optional field
bool maxEndTime_present #true if maxEndTime prediction is given
bool likelyTime_present #true if likelyTime prediction is given
int8 state # state, values as defined below
uint8 DEFAULT = 0
uint8 UNAVAILABLE = 0
uint8 DARK = 1
uint8 STOP___THEN___PROCEED = 2
uint8 STOP___AND___REMAIN = 3
uint8 PRE___MOVEMENT = 4
uint8 PERMISSIVE___MOVEMENT___ALLOWED = 5
uint8 PROTECTED___MOVEMENT___ALLOWED = 6
uint8 PERMISSIVE__CLEARANCE = 7
uint8 PROTECTED__CLEARANCE = 8
uint8 CAUTION___CONFLICTING___TRAFFIC = 9
