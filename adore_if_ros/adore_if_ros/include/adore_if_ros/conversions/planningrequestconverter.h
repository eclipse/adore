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

#include <adore_if_ros_msg/PlanningRequest.h>
#include <adore/fun/tac/planning_request.h>
#include "setpointrequestconverter.h"

namespace adore
{
    namespace if_ROS
    {
        struct PlanningRequestConverter
        {
            SetPointRequestConverter spr_converter_;
            /**
             * Conversion of PlanningRequest to PlanningRequest message
             */
            adore_if_ros_msg::PlanningRequest operator()(const adore::fun::PlanningRequest & data)
            {
                adore_if_ros_msg::PlanningRequest msg;
                msg.iteration = data.iteration;
                msg.t_planning_start = data.t_planning_start;
                msg.t_planning_end = data.t_planning_end;
                msg.t_emergency_start = data.t_emergency_start;
                msg.initial_state = spr_converter_(&data.initial_state);
                return msg;
            }
            /**
             * Conversion of PlanningRequest message to PlanningRequest
             */
            template<typename Tmsg>
            void operator()(Tmsg msg,adore::fun::PlanningRequest& data)
            {
                data.iteration = msg.iteration;
                data.t_planning_start = msg.t_planning_start;
                data.t_planning_end = msg.t_planning_end;
                data.t_emergency_start = msg.t_emergency_start;
                spr_converter_(&msg.initial_state,&data.initial_state);               
            }
        };
    }
}
