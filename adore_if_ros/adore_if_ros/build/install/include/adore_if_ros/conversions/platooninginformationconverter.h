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
 *   Reza Dariani- initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/fun/afactory.h>
#include <adore/fun/platooningInformation.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/CooperativePlanning.h>


namespace adore
{
    namespace if_ROS
    {

        struct PlatooningInformationConverter
        {
            /**
             * Convert a adore_if_ros_msg::PlatooningState to a adore::fun::platooningInformation
             */
            template<typename Tmsg>
            void operator()(Tmsg msg,adore::fun::PlatooningInformation* ps)
            {
                // std::cout<<"\nwrite msg to p_info";
                ps->set(msg);
            }
            /**
             * Convert a adore::fun::platooningInformation to a adore_if_ros_msg::PlatooningState
             */
            adore_if_ros_msg::CooperativePlanning operator()(const adore::fun::PlatooningInformation& ps)
            {
                // std::cout<<"\nwrite p_info to msg";
                adore_if_ros_msg::CooperativePlanning msg;
                msg.tolerated_distance_ahead = ps.getToleratedDistanceAhead();
                msg.tolerated_distance_behind = ps.getToleratedDistanceBehind();
                msg.target_automation_level = ps.getTargetAutomationLevel();
                msg.lane_position = ps.getLanePosition();
                msg.id = ps.getId();
                return msg;
            }
        };
    }
}