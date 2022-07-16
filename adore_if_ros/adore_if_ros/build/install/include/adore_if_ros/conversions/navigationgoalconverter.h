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

#include <adore/fun/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/NavigationGoal.h>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <adore_if_ros_msg/TerminalRequest.h>
#include <adore_if_ros_msg/WheelSpeed.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
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
         * Converts between adore::fun::NavigationGoal and adore_if_ros_msg::NavigationGoal
         */
        struct NavigationGoalConverter
        {
            /// convert a ros navigation goal message to a c++ object
            void operator()(adore_if_ros_msg::NavigationGoalConstPtr msg,adore::fun::NavigationGoal* goal)
            {
                goal->target_.x_ = msg.get()->target.x;
                goal->target_.y_ = msg.get()->target.y;
                goal->target_.z_ = msg.get()->target.z;
            }
        };
    }
}