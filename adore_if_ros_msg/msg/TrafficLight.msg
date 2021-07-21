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
# *  Robert Markowski
# ********************************************************************************
TrafficControlDevice tcd

# unique id describing a related traffic light controller
int32 movementId

# unique id for junction
int32 junctionId

# GREEN = 1, YELLOW = 2, RED = 3, RED_YELLOW = 4, YELLOW_FLASHING = 5, UNDEFINED_COLOR = 0
int16 tlstate