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
#include <adore_if_ros_msg/GapQueue.h>


namespace adore
{
    namespace if_ROS
    {
        struct GapConverter
        {
            /**
             * Conversion  to ros message
             */
            adore_if_ros_msg::GapQueue operator()(const adore::env::GapQueue& value)
            {
                adore_if_ros_msg::GapQueue msg;
                for(const auto& entry:value)
                {
                    msg.data.push_back(this->operator()(entry));
                }
                return msg;   
            }
            adore_if_ros_msg::GapData operator()(const adore::env::GapData& value)
            {
                adore_if_ros_msg::GapData msg;
                msg.rating = value.rating;
                msg.feasible = value.feasible;
                msg.anchor_X = value.anchor_X;
                msg.anchor_Y = value.anchor_Y;
                msg.anchor_Z = value.anchor_Z;
                msg.anchor_dX = value.anchor_dX;
                msg.anchor_dY = value.anchor_dY;
                msg.anchor_dZ = value.anchor_dZ;
                msg.anchor_vt = value.anchor_vt;
                msg.t_obs = value.t_obs;
                msg.lead_exists = value.lead_exists;
                msg.chase_exists = value.chase_exists;
                msg.s_lead = value.s_lead;
                msg.s_chase = value.s_chase;
                msg.v_lead = value.v_lead;
                msg.v_chase = value.v_chase;
                msg.s_anchor = value.s_anchor;
                msg.s_gate_opening = value.s_gate_opening;
                msg.s_gate_closure = value.s_gate_closure;
                return msg;
            }
            void operator()(const adore_if_ros_msg::GapData* msg,adore::env::GapData* value)
            {
                value->rating = msg->rating;
                value->feasible = msg->feasible;
                value->anchor_X = msg->anchor_X;
                value->anchor_Y = msg->anchor_Y;
                value->anchor_Z = msg->anchor_Z;
                value->anchor_dX = msg->anchor_dX;
                value->anchor_dY = msg->anchor_dY;
                value->anchor_dZ = msg->anchor_dZ;
                value->anchor_vt = msg->anchor_vt;
                value->t_obs = msg->t_obs;
                value->lead_exists = msg->lead_exists;
                value->chase_exists = msg->chase_exists;
                value->s_lead = msg->s_lead;
                value->s_chase = msg->s_chase;
                value->v_lead = msg->v_lead;
                value->v_chase = msg->v_chase;
                value->s_anchor = msg->s_anchor;
                value->s_gate_opening = msg->s_gate_opening;
                value->s_gate_closure = msg->s_gate_closure;
            }
            /**
             * Conversion from ros message
             */
            void operator()(adore_if_ros_msg::GapQueueConstPtr msg,adore::env::GapQueue* queue)
            {
                queue->clear();
                for(const auto& entry:msg->data)
                {
                    adore::env::GapData gap;
                    this->operator()(&entry,&gap);
                    queue->push_back(gap);
                }
            }
        };
    }
}
