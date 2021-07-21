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

#include <adore/env/afactory.h>
#include <adore_if_ros_msg/Precedence.h>

namespace adore
{
    namespace if_ROS
    {
        struct PrecedenceRuleConverter
        {
            /**
             * Conversion of adore::env::PrecedenceRule to adore_if_ros_msg::Precedence message
             */
            adore_if_ros_msg::Precedence operator()(const adore::env::PrecedenceRule& rule)
            {
                adore_if_ros_msg::Precedence msg;
                msg.high_priority.first.x = rule.high_.from_(0);
                msg.high_priority.first.y = rule.high_.from_(1);
                msg.high_priority.first.z = rule.high_.from_(2);
                msg.high_priority.last.x = rule.high_.to_(0);
                msg.high_priority.last.y = rule.high_.to_(1);
                msg.high_priority.last.z = rule.high_.to_(2);

                msg.unary = rule.unary_;

                msg.low_priority.first.x = rule.low_.from_(0);
                msg.low_priority.first.y = rule.low_.from_(1);
                msg.low_priority.first.z = rule.low_.from_(2);
                msg.low_priority.last.x = rule.low_.to_(0);
                msg.low_priority.last.y = rule.low_.to_(1);
                msg.low_priority.last.z = rule.low_.to_(2);
                return msg;
            }
            /**
             * Conversion of adore_if_ros_msg::Precedence message to adore::env::PrecedenceRule
             */
            void operator()(adore_if_ros_msg::PrecedenceConstPtr msg,adore::env::PrecedenceRule& rule)
            {
                rule.high_.from_(0) = msg->high_priority.first.x;
                rule.high_.from_(1) = msg->high_priority.first.y;
                rule.high_.from_(2) = msg->high_priority.first.z;
                rule.high_.to_(0) = msg->high_priority.last.x;
                rule.high_.to_(1) = msg->high_priority.last.y;
                rule.high_.to_(2) = msg->high_priority.last.z;

                rule.unary_ = msg->unary;

                rule.low_.from_(0) = msg->low_priority.first.x;
                rule.low_.from_(1) = msg->low_priority.first.y;
                rule.low_.from_(2) = msg->low_priority.first.z;
                rule.low_.to_(0) = msg->low_priority.last.x;
                rule.low_.to_(1) = msg->low_priority.last.y;
                rule.low_.to_(2) = msg->low_priority.last.z;
            }
        };
    }
}



