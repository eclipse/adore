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
#include <tf/LinearMath/Matrix3x3.h>
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
             * Convert adore::sim::ResetVehiclePowe to geometry_msgs::Pose
             */
            geometry_msgs::Pose operator()(const adore::sim::ResetVehiclePose& pose)
            {
                geometry_msgs::Pose msg;
                msg.position.x = pose.getX();
                msg.position.y = pose.getY();
                msg.position.z = pose.getZ();
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, pose.getPSI());
                msg.orientation.w = q.getW();
                msg.orientation.x = q.getX();
                msg.orientation.y = q.getY();
                msg.orientation.z = q.getZ();
                return msg;
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

            /**
             * Convert adore::sim::ResetVehicleTwist to geometry_msgs::Twist
             */
            geometry_msgs::Twist operator()(const adore::sim::ResetVehicleTwist& twist)
            {
                geometry_msgs::Twist msg;
                msg.linear.x = twist.getVx();
                msg.linear.y = twist.getVy();
                msg.angular.z = twist.getOmega();
                return msg;
            }
        };
    }  // namespace if_ROS
}  // namespace adore