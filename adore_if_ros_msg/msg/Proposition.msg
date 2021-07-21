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
uint8 order         #order of calculus, (zero/first), or enum value for others if used
string term         #logical term, e.g. GOAL_REACHED in 0-order
bool value          #boolean value of term, e.g. GOAL_REACHED=true
float64 timeout      #UTC time, after which value shall be considered undefined
bool has_timeout    #if false, timeout shall be considered infinite
