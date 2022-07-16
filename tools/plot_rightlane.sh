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
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/rightlane/targetOuter' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.rightlane.targetOuterBorderDistance.s, y=m.rightlane.targetOuterBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/rightlane/separating' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.rightlane.separatingBorderDistance.s, y=m.rightlane.separatingBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/rightlane/sourceOuter' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.rightlane.sourceOuterBorderDistance.s, y=m.rightlane.sourceOuterBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun adore_if_ros plot_vector_node  '/vehicle0/rightlane/targetOuter' '6' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Right' 's (m)' 'n (m)' __name:=plot_right_targetOuter &
rosrun adore_if_ros plot_vector_node  '/vehicle0/rightlane/separating' '6' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Right' 's (m)' 'n (m)' __name:=plot_right_separating &
rosrun adore_if_ros plot_vector_node  '/vehicle0/rightlane/sourceOuter' '6' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Right' 's (m)' 'n (m)' __name:=plot_right_sourceOuter &

