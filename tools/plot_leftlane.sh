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
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/leftlane/targetOuter' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.leftlane.targetOuterBorderDistance.s, y=m.leftlane.targetOuterBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/leftlane/separating' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.leftlane.separatingBorderDistance.s, y=m.leftlane.separatingBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun topic_tools transform '/vehicle0/ENV/lanegeometry' '/vehicle0/leftlane/sourceOuter' 'adore_if_ros_msg/PointArray' 'adore_if_ros_msg.msg.PointArray(x=m.leftlane.sourceOuterBorderDistance.s, y=m.leftlane.sourceOuterBorderDistance.x, z=[])' --import adore_if_ros_msg &
rosrun adore_if_ros plot_vector_node  '/vehicle0/leftlane/targetOuter' '4' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Left' 's (m)' 'n (m)' __name:=plot_left_targetOuter &
rosrun adore_if_ros plot_vector_node  '/vehicle0/leftlane/separating' '4' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Left' 's (m)' 'n (m)' __name:=plot_left_separating &
rosrun adore_if_ros plot_vector_node  '/vehicle0/leftlane/sourceOuter' '4' 'LineColor=1,0,0; PointSize=5' 'LaneChangeView: Left' 's (m)' 'n (m)' __name:=plot_left_sourceOuter &

