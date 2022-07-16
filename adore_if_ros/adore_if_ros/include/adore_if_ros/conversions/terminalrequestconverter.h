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
         * Converts between adore::fun::TerminalRequest and adore_if_ros_msg::TerminalRequest
         */
        struct TerminalRequestConverter
        {
            /**
             * Convert a adore_if_ros_msg::TerminalRequest to a adore::fun::TerminalRequest
             */
            template<typename Tmsg>
            void operator()(Tmsg msg,adore::fun::TerminalRequest* tr)
            {
                tr->set(msg->X,msg->Y,msg->PSI,msg->t,msg->valid);
            }
            /**
             * Convert a adore::fun::TerminalRequest to a adore_if_ros_msg::TerminalRequest
             */
            adore_if_ros_msg::TerminalRequest operator()(const adore::fun::TerminalRequest& tr)
            {
                adore_if_ros_msg::TerminalRequest msg;
                msg.X = tr.getX();
                msg.Y = tr.getY();
                msg.PSI = tr.getPSI();
                msg.t = tr.getT();
                msg.valid = tr.isValid();
                return msg;
            }
        };
    }
}