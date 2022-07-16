#!/bin/bash
#********************************************************************************
#* Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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
rosrun adore_if_ros plot_graph_node &
sleep 2
rostopic pub -1 /plot adore_if_ros_msg/PlotStart "{tag: 'delta_set', xtopic: '/SIM/utc', ytopic: '/vehicle0/FUN/MotionCommand/steeringAngle', figure: 7, options: 'LineWidth=3;LineColor=1,0,0'}"