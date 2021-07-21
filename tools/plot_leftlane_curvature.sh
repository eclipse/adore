#!/bin/bash
#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Daniel Heß
#********************************************************************************
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/leftlane_curvature' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.lefttargetlane.centerCurvature.s, y=m.lefttargetlane.centerCurvature.x, z=[])' --import adore_if_ros_msg &
rosrun adore_if_ros plot_vector_node  '/vehicle0/leftlane_curvature' '3' 'LineColor=0.7,0.7,0; PointSize=5' 'Lane Curvature' 's (m)' 'kappa (1/m)' __name:=plot_leftcurvature