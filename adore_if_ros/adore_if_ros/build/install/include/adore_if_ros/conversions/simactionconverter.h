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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#include <adore/sim/action.h>
#include <adore_if_ros_msg/Action.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Convert between adore::sim::SchedulerNotification and ROS adore_if_ros_msg::SchedulerNotification
         */
        struct SimActionConverter
        {
            void operator()(adore_if_ros_msg::ActionConstPtr msg, adore::sim::Action& a)
            {
                a.setAction(msg->action);
            }
            void operator()(const adore_if_ros_msg::Action& msg, adore::sim::Action& object)
            {
                object.setAction(msg.action);
            }
            adore_if_ros_msg::Action operator()(const adore::sim::Action& a)
            {
                adore_if_ros_msg::Action msg;
                msg.action = a.getAction();
                return msg;
            }
        };
    }
}