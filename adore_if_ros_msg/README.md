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
    * /vehicle0/VEH/steering_angle_measured
* wheel speeds (speed above ground in m/s), FL,RL,RR,FR
    * std_msgs::Float32MultiArray
    * /vehicle0/VEH/wheel_speed
* automatic control enabled
    * std_msgs::Bool
    * /vehicle0/VEH/AutomaticControlState/acceleration
    * /vehicle0/VEH/AutomaticControlState/accelerationActive
    * /vehicle0/VEH/AutomaticControlState/steering
* yaw-rate measured by ESP (rad/s)
    * std_msgs::Float32
    * /vehicle0/VEH/yaw_rate
* acceleration measured by ESP (m/s², ax, ay, az) 
    * std_msgs::Float32
    * /vehicle0/VEH/ax
* indicator state: left/right
    * std_msgs::Bool
    * /vehicle0/VEH/IndicatorState/left
* gear state (0:Park, 1: Drive, 2: Reverse, 3: Neutral): 
    * std_msgs::Int8
    * /vehicle0/VEH/gear_state

## From adore to vehicle base system
* control input: longitudinal acceleration request (m/s²): 
    * std_msgs::Float32
    * /vehicle0/FUN/MotionCommand/acceleration
* control input: steering wheel angle request (rad): 
    * std_msgs::Float32
    * /vehicle0/FUN/MotionCommand/steeringAngle
* indicator input: left/right 
    * std_msgs::Bool
    * /vehicle0/FUN/IndicatorCommand/left
* gear request (0:Park, 1: Drive, 2: Reverse, 3: Neutral): 
    * std_msgs::Int8
    * /vehicle0/FUN/GearSelectionCommand

## From sensors to adore
* ego state estimate (Position xyz UTM (current tile) in m, Rotation around xyz in rad, velocity linear in vehicle coordinates m/s, velocity rotational rad/s): 
    * nav_msgs::Odometry
    * /vehicle0/odom
* observation of other traffic participants
    * object oriented information: Specify geometric state, type, indicator/lights
    * adore_if_ros_msgs::TrafficParticipantSet
    * /vehicle0/traffic

## adore internal
* ENV (environment representations)
    * road marking/ drivable lane: Border
        * adore_if_ros_msg::Border
        * /vehicle0/ENV/Border
    * traffic light, traffic sign, precedence rules
        * static precedence rule
            * adore_if_ros_msg::Precedence
            * /vehicle0/ENV/Precedence
        * tcd controlled state of a connection
            * adore_if_ros_msg::TCDConnectionState
            * /vehicle0/ENV/TCDConnectionState
    * parking spot: ParkingSpot, tbd
    * logical propositions about environment and ego state
        * adore_if_ros_msg::Proposition
        * /vehicle0/ENV/Propositions
    * prediction of space occupied by other traffic participants in the future
        * adore_if_ros_msg::OccupancyCylinderPredictionSet
        * /vehicle0/ENV/Prediction/expected - the expected behavior
        * /vehicle0/ENV/Prediction/worstcase - the worst-case behavior
        * /vehicle0/ENV/Prediction/desired - the desired behavior of an agent
    * space that may be affected by ego vehicle
        * adore:if_ros_msg::AreaOfEffect
        * /vehicle0/ENV/areaofeffect
* FUN (control functions)
    * Trajectory to be executed
        * adore_if_ros_msg::SetPointRequest
        * /vehicle0/FUN/SetPointRequest
    * Nominal Trajectory, which could be executed if emergency maneuver is not triggered
        * adore_if_ros_msg::SetPointRequest
        * /vehicle0/FUN/NominalTrajectory
    * low level emergency breaking: 
        * adore_if_ros_msg::TerminalRequest
        * /vehicle0/FUN/TerminalRequest
    * navigation goal: current target point for navigation node
        * adore_if_ros_msg::NavigationGoal
        * /vehicle0/FUN/NavigationGoal
    * Request to plan trajectories 
        * adore_if_ros_msg::PlanningRequest
        * /vehicle0/FUN/PlanningRequest
    * Trajectory planning result, reply to PlanningRequest
        * adore_if_ros_msg::PlanningResult
        * /vehicle0/FUN/PlanningResult

## Simulation
* simulation time
    * std_msgs::Float64
    * /SIM/utc
* state information of all traffic participants in the simulation
    * TrafficParticipantSimulation
    * /SIM/traffic, /SIM/traffic/agg
* reset vehicle state
    * geometry_msgs::Pose
    * /vehicle0/SIM/ResetVehiclePose
    * geometry_msgs::Twist
    * /vehicle0/SIM/ResetVehicleTwist
