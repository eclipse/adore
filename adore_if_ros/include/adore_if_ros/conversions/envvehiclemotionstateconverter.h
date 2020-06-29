/********************************************************************************
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
 ********************************************************************************/

#pragma once
#include <math.h>
#include <tf/tf.h>
#include "quaternion.h"
#include <adore/env/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/Border.h>
#include <adore_if_ros_msg/NavigationGoal.h>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <adore_if_ros_msg/TerminalRequest.h>
#include <adore_if_ros_msg/WheelSpeed.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Conversions between adore::env::VehicleMotionState9d and ROS messages.
         */
        struct VehicleMotionStateConverter
        {
            public:
            /**
             * Converts a ROS odometry message into a VehicleMotionState9d object.
             * Note that steering angle and longitudinal acceleration are not filled.
             */
            void operator()(nav_msgs::OdometryConstPtr msg, adore::env::VehicleMotionState9d * state)
            {
                state->setTime(msg->header.stamp.toSec());
                state->setX(msg->pose.pose.position.x);
                state->setY(msg->pose.pose.position.y);
                state->setZ(msg->pose.pose.position.z);
                QuaternionConverter qc;
                state->setPSI(qc.quaternionToHeading(msg->pose.pose));
                state->setvx(msg->twist.twist.linear.x);
                state->setvy(msg->twist.twist.linear.y);
                state->setOmega(msg->twist.twist.angular.z);
            }
        };




    }
}