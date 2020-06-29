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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Short hand for often used tf2::Quaternion to/from heading conversion
         */
        struct QuaternionConverter
        {
            public:
            ///set heading of a Pose message
            void setHeading(double heading, geometry_msgs::Pose& msg)const
            {
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,heading);       
                         
                msg.orientation.x = q.getX();
                msg.orientation.y = q.getY();
                msg.orientation.z = q.getZ();
                msg.orientation.w = q.getW();
            } 
            ///convert heading to quaternion
            tf2::Quaternion headingToQuaternion(double heading)const 
            {
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,heading);                
                return q;
            }
            ///convert quaternion to heading
            double quaternionToHeading(const geometry_msgs::Pose& p)const
            {
                tf2::Quaternion q;
                q.setValue(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll,pitch,yaw;
                m.getRPY(roll,pitch,yaw);
                return yaw;
            }
        };
    }
}