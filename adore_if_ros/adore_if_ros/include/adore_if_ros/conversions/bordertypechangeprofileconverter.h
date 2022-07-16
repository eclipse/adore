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
#include <adore_if_ros_msg/BorderTypeChangeProfile.h>
#include <adore/env/map/map_border_management.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Converter between adore::env::BorderTypeChangeProfile and adore_if_ros_msg::BorderTypeChangeProfile
         */
        struct BorderTypeChangeProfileConverter
        {
            public:
            /**
             * Conversion of adore_if_ros_msg::Border to adore::env::BorderBased::Border
             */
            void operator()(adore_if_ros_msg::BorderTypeChangeProfileConstPtr msg,adore::env::BorderTypeChangeProfile& btcp)
            {
                btcp.start.m_X  = msg->start.x;
                btcp.start.m_Y  = msg->start.y;
                btcp.start.m_Z  = msg->start.z;
                btcp.end.m_X  = msg->end.x;
                btcp.end.m_Y  = msg->end.y;
                btcp.end.m_Z  = msg->end.z;
                for(const auto & t : msg->bordertypeprofile)
                {
                    btcp.borderTypeProfile.push_back(static_cast<adore::env::BorderBased::BorderType::TYPE>(t));
                }
            }

            adore_if_ros_msg::BorderTypeChangeProfile operator()(const adore::env::BorderTypeChangeProfile & btcp)
            {
                adore_if_ros_msg::BorderTypeChangeProfile msg;
                msg.start.x = btcp.start.m_X;
                msg.start.y = btcp.start.m_Y;
                msg.start.z = btcp.start.m_Z;

                msg.end.x = btcp.end.m_X;
                msg.end.y = btcp.end.m_Y;
                msg.end.z = btcp.end.m_Z;

                for(const auto & t : btcp.borderTypeProfile)
                {
                    msg.bordertypeprofile.push_back(static_cast<int>(t));
                }

                return msg;
            }
        };
    }
}