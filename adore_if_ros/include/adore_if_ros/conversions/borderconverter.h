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
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
# include <adore/env/afactory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <adore_if_ros_msg/Border.h>
#include <nav_msgs/Odometry.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         *  Conversion between adore::env::BorderBased::Border and adore_if_ros_msg::Border.
         */
        struct BorderConverter
        {
            public:
            /**
             * Conversion of adore_if_ros_msg::Border to adore::env::BorderBased::Border
             */
            void operator()(adore_if_ros_msg::BorderConstPtr msg,adore::env::BorderBased::Border& border)
            {
            //border id: first node
            border.m_id.m_first.m_X = msg->borderId.first.x;
            border.m_id.m_first.m_Y = msg->borderId.first.y;
            border.m_id.m_first.m_Z = msg->borderId.first.z;
            //border id: last node
            border.m_id.m_last.m_X = msg->borderId.last.x;
            border.m_id.m_last.m_Y = msg->borderId.last.y;
            border.m_id.m_last.m_Z = msg->borderId.last.z;
            //path
            border.m_path = new adore::mad::LLinearPiecewiseFunctionM<double,3>(msg->points.size()+2,0.0);
            border.m_path->getData()(0,0) = 0;
            border.m_path->getData()(1,0) = border.m_id.m_first.m_X;
            border.m_path->getData()(2,0) = border.m_id.m_first.m_Y;
            border.m_path->getData()(3,0) = border.m_id.m_first.m_Z;
            double dx,dy,dz;
            int counter = 1;
            for(const auto& p:msg->points)
            {
                border.m_path->getData()(1,counter) = p.x;
                border.m_path->getData()(2,counter) = p.y;
                border.m_path->getData()(3,counter) = p.z;
                dx = border.m_path->getData()(1,counter) - border.m_path->getData()(1,counter-1);
                dy = border.m_path->getData()(2,counter) - border.m_path->getData()(2,counter-1);
                dz = border.m_path->getData()(3,counter) - border.m_path->getData()(3,counter-1);
                border.m_path->getData()(0,counter) = border.m_path->getData()(0,counter-1) + std::sqrt(dx*dx+dy*dy+dz*dz);
                counter ++;
            }
            border.m_path->getData()(1,counter) = border.m_id.m_last.m_X;
            border.m_path->getData()(2,counter) = border.m_id.m_last.m_Y;
            border.m_path->getData()(3,counter) = border.m_id.m_last.m_Z;
            dx = border.m_path->getData()(1,counter) - border.m_path->getData()(1,counter-1);
            dy = border.m_path->getData()(2,counter) - border.m_path->getData()(2,counter-1);
            dz = border.m_path->getData()(3,counter) - border.m_path->getData()(3,counter-1);
            border.m_path->getData()(0,counter) = border.m_path->getData()(0,counter-1) + std::sqrt(dx*dx+dy*dy+dz*dz);

            border.m_left = 0;
            if(msg->has_left_id)
            {
                border.m_left = new adore::env::BorderBased::BorderID();
                //border id: first node
                border.m_left->m_first.m_X = msg->leftId.first.x;
                border.m_left->m_first.m_Y = msg->leftId.first.y;
                border.m_left->m_first.m_Z = msg->leftId.first.z;
                //border id: last node
                border.m_left->m_last.m_X = msg->leftId.last.x;
                border.m_left->m_last.m_Y = msg->leftId.last.y;
                border.m_left->m_last.m_Z = msg->leftId.last.z; 
            }
            border.m_type = static_cast<adore::env::BorderBased::BorderType::TYPE>(msg->borderType);
            }

            /**
             * Conversion of adore::env::BorderBasedBorder::Border to adore_if_ros_msg::Border
             */
            adore_if_ros_msg::Border operator()(const adore::env::BorderBased::Border & border)
            {
            adore_if_ros_msg::Border msg;
            // border id
            geometry_msgs::Point first;
            first.x = border.m_id.m_first.m_X;
            first.y = border.m_id.m_first.m_Y;
            first.z = border.m_id.m_first.m_Z;
            msg.borderId.first = first;

            geometry_msgs::Point last;
            last.x = border.m_id.m_last.m_X;
            last.y = border.m_id.m_last.m_Y;
            last.z = border.m_id.m_last.m_Z;
            msg.borderId.last = last;
            
            // path - write all elements except first and last
            if( border.m_path != nullptr)
            {
                for(int i = 1; i < border.m_path->getData().nc() -1; i++)
                {
                geometry_msgs::Point p;
                p.x = border.m_path->getData()(1,i);
                p.y = border.m_path->getData()(2,i);
                p.z = border.m_path->getData()(3,i);
                msg.points.push_back(p);
                }
            }

            // left
            if(border.m_left != nullptr)
            {
                msg.has_left_id = true;
                geometry_msgs::Point left_first;
                left_first.x = border.m_left->m_first.m_X;
                left_first.y = border.m_left->m_first.m_Y;
                left_first.z = border.m_left->m_first.m_Z;
                msg.leftId.first = left_first;

                geometry_msgs::Point left_last;
                left_last.x = border.m_left->m_last.m_X;
                left_last.y = border.m_left->m_last.m_Y;
                left_last.z = border.m_left->m_last.m_Z;
                msg.leftId.last = left_last;
            }
            else
            {
                msg.has_left_id = false;
            }
            msg.borderType = border.m_type;
            return msg;
            }
        };
    }
}