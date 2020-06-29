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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore_if_ros_msg/NavigationData.h>
#include <adore/env/borderbased/borderid.h>

namespace adore
{
    namespace if_ROS
    {
        struct NavigationDataConverter
        {
            /**
             * Conversion of pair<BorderID,double(cost)> to adore_if_ros_msg::NavigationData message
             */
            adore_if_ros_msg::NavigationData operator()(const std::pair<adore::env::BorderBased::BorderID,double> & id2cost)
            {
                adore_if_ros_msg::NavigationData msg;
                
                msg.borderId.first.x = id2cost.first.m_first.m_X;
                msg.borderId.first.y = id2cost.first.m_first.m_Y;
                msg.borderId.first.z = id2cost.first.m_first.m_Z;
                msg.borderId.last.x = id2cost.first.m_last.m_X;
                msg.borderId.last.y = id2cost.first.m_last.m_Y;
                msg.borderId.last.z = id2cost.first.m_last.m_Z;
                
                msg.cost = id2cost.second;

                return msg;
            }
            /**
             * Conversion of adore_if_ros_msg::NavigationData message to pair<BorderID,double(cost)>
             */
            void operator()(adore_if_ros_msg::NavigationDataConstPtr msg,std::pair<adore::env::BorderBased::BorderID,double>& id2cost)
            {
                id2cost.first.m_first.m_X = msg->borderId.first.x;
                id2cost.first.m_first.m_Y = msg->borderId.first.y;
                id2cost.first.m_first.m_Z = msg->borderId.first.z;
                id2cost.first.m_last.m_X = msg->borderId.last.x;
                id2cost.first.m_last.m_Y = msg->borderId.last.y;
                id2cost.first.m_last.m_Z = msg->borderId.last.z;

                id2cost.second = msg->cost;
            }
        };
    }
}
