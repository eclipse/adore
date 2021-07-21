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
float64 time # time of state
nav_msgs/Odometry motion_state
shape_msgs/SolidPrimitive shape
float64 dimensions_variance # variance of dimensions of shape
TrafficClassification classification
uint8 classification_certainty	# confidence of classification 0..100
uint8 existance_certainty # confidence that traffic participant exists 0..100
bool leftIndicatorOn # indicator in yaw direction on the left side
uint8 leftIndicator_certainty # confidence of detection of left indicator state 0..100
bool rightIndicatorOn # indicator in yaw direction on the right side
uint8 rightIndicator_certainty # confidence of detection of right indicator state 0..100
bool brakeLightOn # state of the brake lights
uint8 brakeLight_certainty# confidence of detection of brake light state 0..100
bool lowBeamOn # state of the low beam
uint8 lowBeam_certainty# confidence of detection of low beam state 0..100
bool highBeamOn # state of the high beam
uint8 highBeam_certainty# confidence of detection of high beam state 0..100
uint64 v2xStationID #etsi Vehicle to X id
uint8 v2xStationID_certainty# confidence of association of v2xStationID

# certainty scaling
uint8 certainty_max=100
