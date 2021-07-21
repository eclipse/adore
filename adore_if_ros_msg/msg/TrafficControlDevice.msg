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
geometry_msgs/Pose pose

# types: TRAFFIC_LIGHT = 0,
#        TRAFFIC_LIGHT_PEDESTRIAN = 1,
#        RIGHT_OF_WAY = 2, 
#        STOP_SIGN = 3,
#        SPEED_LIMIT_10 = 4,
#        SPEED_LIMIT_20 = 5,
#        SPEED_LIMIT_30 = 6,
#        SPEED_LIMIT_40 = 7,
#        SPEED_LIMIT_50 = 8,
#        SPEED_LIMIT_60 = 9,
#        SPEED_LIMIT_70 = 10,
#        SPEED_LIMIT_80 = 11,
#        SPEED_LIMIT_90 = 12,
#        SPEED_LIMIT_100 = 13,
#        SPEED_LIMIT_120 = 14,
#        SPEED_LIMIT_130 = 15,
#        SPEED_LIMIT_NON = 16,
#        CITY_LIMIT_SIGN = 17,
#        MOTOR_WAY_SIGN = 18,
#        UNKNOWN = 99
int16 type

int32 id

# borders / lanes that are related to tcd
BorderID[] relatedBorders