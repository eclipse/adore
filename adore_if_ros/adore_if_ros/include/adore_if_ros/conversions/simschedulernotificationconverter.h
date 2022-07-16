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

#include <adore/sim/schedulernotification.h>
#include <adore_if_ros_msg/SchedulerNotification.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Convert between adore::sim::SchedulerNotification and ROS adore_if_ros_msg::SchedulerNotification
         */
        struct SimSchedulerNotificationConverter
        {
            void operator()(adore_if_ros_msg::SchedulerNotificationConstPtr msg, adore::sim::SchedulerNotification& sn)
            {
                sn.setID(msg->identifier);
                sn.setUpperTimeLimit(msg->upperTimeLimit.sec,msg->upperTimeLimit.nsec);
               // sn.setUpperTimeLimit(msg->upperTimeLimit)
            }
            void operator()(const adore_if_ros_msg::SchedulerNotification& sn, adore::sim::SchedulerNotification& object)
            {
                object.setID(sn.identifier);
                object.setUpperTimeLimit(sn.upperTimeLimit.sec,sn.upperTimeLimit.nsec);
            }
            adore_if_ros_msg::SchedulerNotification operator()(const adore::sim::SchedulerNotification& sn)
            {
                adore_if_ros_msg::SchedulerNotification sncp;
                sncp.upperTimeLimit.sec = sn.getUpperTimeLimit_sec();
                sncp.upperTimeLimit.nsec = sn.getUpperTimeLimit_nsec();
                sncp.identifier = sn.getID();
                return sncp;
            }
        };
    }
}