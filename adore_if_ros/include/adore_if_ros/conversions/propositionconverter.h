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

#include <adore_if_ros_msg/Proposition.h>
#include <adore/env/situation/proposition.h>

namespace adore
{
    namespace if_ROS
    {
        struct PropositionConverter
        {

            // unsigned char order_;   /**< order of calculus, (zero/first), or enum value for others if used */
            // std::string term_;      /**< logical term, e.g. GOAL_REACHED in 0-order */
            // bool value_;            /**< boolean value of term, e.g. GOAL_REACHED=true */
            // double timeout_;        /**< UTC time, after which value shall be considered undefined */
            // bool has_timeout_;      /**< if false, timeout shall be considered infinite */

            // uint8 order         #order of calculus, (zero/first), or enum value for others if used
            // string term         #logical term, e.g. GOAL_REACHED in 0-order
            // bool value          #boolean value of term, e.g. GOAL_REACHED=true
            // double timeout      #UTC time, after which value shall be considered undefined
            // bool has_timeout    #if false, timeout shall be considered infinite

            /**
             * Conversion of adore::env::Proposition to adore_if_ros_msg::Proposition message
             */
            adore_if_ros_msg::Proposition operator()(const adore::env::Proposition & p)
            {
                adore_if_ros_msg::Proposition msg;
                
                msg.order=p.order_;
                msg.term=p.term_;
                msg.value=p.value_;
                msg.timeout=p.timeout_;
                msg.has_timeout=p.has_timeout_;

                return msg;
            }
            /**
             * Conversion of adore_if_ros_msg::Proposition message to adore::env::Proposition
             */
            void operator()(adore_if_ros_msg::PropositionConstPtr msg,adore::env::Proposition& p)
            {
                p.order_=msg->order;
                p.term_=msg->term;
                p.value_=msg->value;
                p.timeout_=msg->timeout;
                p.has_timeout_=msg->has_timeout;
            }
        };
    }
}
