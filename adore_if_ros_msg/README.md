<!--
********************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Daniel Heß - initial API and implementation
********************************************************************************
-->

## Message definitions for ROS interface of adore
In order to communicate with ADORe in ROS this package should be used. By separating adore_if_ros and adore_if_ros_msg, interacting components are independent from adore source code.
Message types from common_msgs and std_msgs are preferredly employed for communication with external modules. 
Internal messages will be mainly specified as user defined messages.
All message headers shall be time-stamped with the true UTC time from the GPS module (or simulation equivalent).

## From vehicle base system to adore
* measured steering wheel angle in rad: 
    * std_msgs::Float32
    * /vehicleXY/VEH/steering_angle_measured
* wheel speeds (speed above ground in m/s), FL,RL,RR,FR
    * std_msgs::Float32MultiArray
    * /vehicleXY/VEH/wheel_speed
* automatic control enabled
    * std_msgs::Bool
    * /vehicleXY/VEH/automation_enabled
* yaw-rate measured by ESP (rad/s)
    * std_msgs::Float32
    * /vehicleXY/VEH/yaw_rate
* acceleration measured by ESP (m/s², ax, ay, az) 
    * std_msgs::Float32
    * /vehicleXY/VEH/ax
* indicator state: left/right
    * std_msgs::Bool
    * /vehicleXY/VEH/IndicatorState/left
* gear state (0:Park, 1: Drive, 2: Reverse, 3: Neutral): 
    * std_msgs::Int8
    * /vehicleXY/VEH/gear_state

## From adore to vehicle base system
* control input: longitudinal acceleration request (m/s²): 
    * std_msgs::Float32
    * /vehicleXY/FUN/MotionCommand/acceleration
* control input: steering wheel angle request (rad): 
    * std_msgs::Float32
    * /vehicleXY/FUN/MotionCommand/steeringAngle
* indicator input: left/right 
    * std_msgs::Bool
    * /vehicleXY/FUN/IndicatorCommand/left
* gear request (0:Park, 1: Drive, 2: Reverse, 3: Neutral): 
    * std_msgs::Int8
    * /vehicleXY/FUN/GearSelectionCommand

## From sensors to adore
* ego state estimate (Position xyz UTM (current tile) in m, Rotation around xyz in rad, velocity linear in vehicle coordinates m/s, velocity rotational rad/s): 
    * nav_msgs::Odometry
    * /vehicleXY/odom
* observation of other traffic participants
    * object oriented information: Specify geometric state, type, indicator/lights
    * adore_if_ros_msgs::TrafficParticipantSet
    * /vehicleXY/traffic

## adore internal
* ENV (environment representations)
    * road marking/ drivable lane: Border
        * adore_if_ros_msg::Border
        * /vehicleXY/ENV/Border
    * traffic light, traffic sign, precedence rules
        * static precedence rule
            * adore_if_ros_msg::Precedence
            * /vehicleXY/ENV/Precedence
        * tcd controlled state of a connection
            * adore_if_ros_msg::TCDConnectionState
            * /vehicleXY/ENV/TCDConnectionState
    * parking spot: ParkingSpot, tbd
    * logical propositions about environment and ego state
        * adore_if_ros_msg::Proposition
        * /vehicleXY/ENV/Propositions
* FUN (control functions)
    * trajectory with feed forward controls: SetPointRequest
        * adore_if_ros_msg::SetPointRequest
        * /vehicleXY/FUN/SetPointRequest
    * emergency breaking: 
        * adore_if_ros_msg::TerminalRequest
        * /vehicleXY/FUN/TerminalRequest
    * navigation goal: current target point for navigation node
        * adore_if_ros_msg::NavigationGoal
        * /vehicleXY/FUN/NavigationGoal

## Simulation
* simulation time
    * std_msgs::Float64
    * /SIM/utc
* state information of all traffic participants in the simulation
    * TrafficParticipantSimulation
    * /SIM/traffic, /SIM/traffic/agg
* reset vehicle state
    * geometry_msgs::Pose
    * /vehicleXY/SIM/ResetVehiclePose
    * geometry_msgs::Twist
    * /vehicleXY/SIM/ResetVehicleTwist
