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
# *  Stephan Lapoehn
# ********************************************************************************
TrafficLight tl

int16 redDuration

int16 yellowDuration

int16 greenDuration

int16 yellowRedDuration

# GREEN = 1, YELLOW = 2, RED = 3, RED_YELLOW = 4, YELLOW_FLASHING = 5, UNDEFINED_COLOR = 0
int16 startState

int16 timeToRed

int16 timeToRedYellow

int16 timeToYellow

int16 timeToGreen
