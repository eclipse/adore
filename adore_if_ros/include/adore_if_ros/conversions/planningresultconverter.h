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

#include <adore_if_ros_msg/PlanningResultSet.h>
#include <adore/fun/tac/planning_result.h>
#include "setpointrequestconverter.h"
#include "terminalrequestconverter.h"
#include "occupancyconverter.h"

namespace adore
{
    namespace if_ROS
    {
        struct PlanningResultConverter
        {
            SetPointRequestConverter spr_converter_;
            TerminalRequestConverter tr_converter_;
            // /**
            //  * Conversion of TPlanningResultSet to PlanningResultSet message
            //  */
            // adore_if_ros_msg::PlanningResultSet operator()(const adore::fun::TPlanningResultSet & result_set)
            // {
            //     adore_if_ros_msg::PlanningResultSet msg;
            //     for(auto& result:result_set)
            //     {
            //         msg.data.push_back((*this)(result));
            //     }
            //     return msg;
            // }
            // /**
            //  * Conversion of PlanningResultSet message to TPlanningResultSet
            //  */
            // void operator()(adore_if_ros_msg::PlanningResultSetConstPtr msg,adore::fun::TPlanningResultSet* result_set)
            // {
            //     result_set->clear();
            //     for(auto& m:msg->data)
            //     {
            //         adore::fun::PlanningResult result;
            //         (*this)(&m,&result);
            //         result_set->push_back(result);
            //     }
            // }
            /**
             * Conversion of TPlanningResult to PlanningResult message
             */
            adore_if_ros_msg::PlanningResult operator()(const adore::fun::PlanningResult & result)
            {
                adore_if_ros_msg::PlanningResult msg;
                msg.id = result.id;
                msg.iteration = result.iteration;
                msg.name = result.name;
                msg.nominal_maneuver = spr_converter_(result.nominal_maneuver);
                msg.combined_maneuver = spr_converter_(result.combined_maneuver);
                msg.terminal_maneuver = tr_converter_(result.terminal_maneuver);
                msg.nominal_maneuver_valid = result.nominal_maneuver_valid;
                msg.combined_maneuver_valid = result.combined_maneuver_valid;
                msg.status_string = result.status_string;
                msg.maneuver_type = result.maneuver_type;
                msg.indicator_left = result.indicator_left;
                msg.indicator_right = result.indicator_right;
                for(auto [cp_name,cp_value] : result.objective_values)
                {
                    adore_if_ros_msg::CostPair cpmsg;
                    cpmsg.objective_name = cp_name;
                    cpmsg.objective_value = cp_value;
                    msg.objective_values.push_back(cpmsg);
                }
                for(auto [cp_name,cp_value] :result.performance_values)
                {
                    adore_if_ros_msg::CostPair cpmsg;
                    cpmsg.objective_name = cp_name;
                    cpmsg.objective_value = cp_value;
                    msg.performance_values.push_back(cpmsg);
                }
                //copy the nominal_maneuver_swath
                msg.nominal_maneuver_swath.trackingID = 0;
                msg.nominal_maneuver_swath.branchID = 0;
                msg.nominal_maneuver_swath.predecessorID = 0;
                msg.nominal_maneuver_swath.confidence = 0;
                for(const auto& pair: result.nominal_maneuver_swath.getLevel(0))
                {
                    const auto& cylinder = pair.second;
                    adore_if_ros_msg::OccupancyCylinder msgi;
                    msgi.rxy = cylinder.rxy_;
                    msgi.x = cylinder.x_;
                    msgi.y = cylinder.y_;
                    msgi.t0 = cylinder.t0_;
                    msgi.t1 = cylinder.t1_;
                    msgi.z0 = cylinder.z0_;
                    msgi.z1 = cylinder.z1_;
                    msg.nominal_maneuver_swath.occupancy.push_back(msgi);
                }
                //copy the combined_maneuver_swath
                msg.combined_maneuver_swath.trackingID = 0;
                msg.combined_maneuver_swath.branchID = 0;
                msg.combined_maneuver_swath.predecessorID = 0;
                msg.combined_maneuver_swath.confidence = 0;
                for(const auto& pair: result.combined_maneuver_swath.getLevel(0))
                {
                    const auto& cylinder = pair.second;
                    adore_if_ros_msg::OccupancyCylinder msgi;
                    msgi.rxy = cylinder.rxy_;
                    msgi.x = cylinder.x_;
                    msgi.y = cylinder.y_;
                    msgi.t0 = cylinder.t0_;
                    msgi.t1 = cylinder.t1_;
                    msgi.z0 = cylinder.z0_;
                    msgi.z1 = cylinder.z1_;
                    msg.combined_maneuver_swath.occupancy.push_back(msgi);
                }

                return msg;
            }
            /**
             * Conversion of PlanningResult message to PlanningResult
             */
            template<typename Tmsg>
            void operator()(Tmsg msg,adore::fun::PlanningResult& result)
            {
                result.id = msg->id;
                result.iteration = msg->iteration;
                result.name = msg->name;
                spr_converter_(&msg->nominal_maneuver,&result.nominal_maneuver);
                spr_converter_(&msg->combined_maneuver,&result.combined_maneuver);
                tr_converter_(&msg->terminal_maneuver,&result.terminal_maneuver);
                result.nominal_maneuver_valid = msg->nominal_maneuver_valid;
                result.combined_maneuver_valid = msg->combined_maneuver_valid;
                result.status_string = msg->status_string;
                result.maneuver_type = msg->maneuver_type;
                result.indicator_left = msg->indicator_left;
                result.indicator_right = msg->indicator_right;
                for(auto [name, value]:msg->objective_values)
                {
                   result.objective_values.insert({name,value});
                }
                for(auto [name, value]:msg->performance_values)
                {
                    result.performance_values.insert({name,value});
                }
                //copy nominal_maneuver_swath
                for(auto& msgi:msg->nominal_maneuver_swath.occupancy)
                {
                    adore::mad::OccupancyCylinder cylinder;
                    cylinder.rxy_ = msgi.rxy;
                    cylinder.x_ = msgi.x;
                    cylinder.y_ = msgi.y;
                    cylinder.t0_ = msgi.t0;
                    cylinder.t1_ = msgi.t1;
                    cylinder.z0_ = msgi.z0;
                    cylinder.z1_ = msgi.z1;
                    result.nominal_maneuver_swath.insert(cylinder);
                }
                //copy combined_maneuver_swath
                for(auto& msgi:msg->combined_maneuver_swath.occupancy)
                {
                    adore::mad::OccupancyCylinder cylinder;
                    cylinder.rxy_ = msgi.rxy;
                    cylinder.x_ = msgi.x;
                    cylinder.y_ = msgi.y;
                    cylinder.t0_ = msgi.t0;
                    cylinder.t1_ = msgi.t1;
                    cylinder.z0_ = msgi.z0;
                    cylinder.z1_ = msgi.z1;
                    result.combined_maneuver_swath.insert(cylinder);
                }
            }
        };
    }
}
