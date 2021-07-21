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

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>




namespace adore
{
    namespace if_ROS
    {
        /**
         * Provides conversion methods for ROS standard data types.
         */
        struct StdConverter
        {
            public:
                std_msgs::Float64 operator()(double value)
                {
                    std_msgs::Float64 m;
                    m.data = value;
                    return m;
                }
                void operator()(std_msgs::Float64ConstPtr msg, double* value)
                {
                    *value = msg.get()->data;
                }

                std_msgs::String operator()(std::string value)
                {
                    std_msgs::String m;
                    m.data = value;
                    return m;
                }
                void operator()(std_msgs::StringConstPtr msg, std::string* value)
                {
                    *value = msg.get()->data;
                }
                void operator()(std_msgs::StringConstPtr msg, std::string& value)
                {
                    value = msg.get()->data;
                }
                void operator()(const std_msgs::String& msg_string, std::string& std_string)
                {
                    std_string = msg_string.data;
                }

        };
    }

}