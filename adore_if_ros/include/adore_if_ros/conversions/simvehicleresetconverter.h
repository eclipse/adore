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

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <adore/sim/resetvehiclepose.h>
#include <adore/sim/resetvehicletwist.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Convert between adore::sim::ResetVehicle(Pose/Twist) and ROS geometry_msgs::(Pose/Twist).
         */
        struct SimVehicleResetConverter
        {
            public:
            /**
             * Convert geometry_msgs::Pose to adore::sim::ResetVehiclePose
             */
            void operator()(geometry_msgs::PoseConstPtr msg,adore::sim::ResetVehiclePose& pose)
            {
                double r,p,y;
                tf::Matrix3x3(tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w)).getRPY(r,p,y);
                pose.setX(msg->position.x);
                pose.setY(msg->position.y);
                pose.setZ(msg->position.z);
                pose.setPSI(y);
            }
            /**
             * Convert geometry_msgs::Twist to adore::sim::ResetVehicleTwist
             */
            void operator()(geometry_msgs::TwistConstPtr msg,adore::sim::ResetVehicleTwist& twist)
            {
                twist.setVx(msg->linear.x);
                twist.setVy(msg->linear.y);
                twist.setOmega(msg->angular.z);
            }

        };
    }
}