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
bool isValid
float64 gate_s0 #gate start, parametrized according to target lane baseline
float64 gate_s1 #gate end, parametrized according to target lane baseline
int8 direction #lane change direction
int8 LEFT = 1
int8 RIGHT = -1
LinearPiecewiseFunction1d targetOuterBorderDistance #border on the outside of the target lane
LinearPiecewiseFunction1d separatingBorderDistance  #the border separating both lanes
LinearPiecewiseFunction1d sourceOuterBorderDistance #border on the outside of the source lane