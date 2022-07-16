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
            // void operator()(adore_if_ros_msg::PlanningResultSetConstPtr msg,adore::fun::TPlanningResultSet*
            // result_set)
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
            adore_if_ros_msg::PlanningResult operator()(const adore::fun::PlanningResult& result)
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
                for (auto [cp_name, cp_value] : result.objective_values)
                {
                    adore_if_ros_msg::CostPair cpmsg;
                    cpmsg.objective_name = cp_name;
                    cpmsg.objective_value = cp_value;
                    msg.objective_values.push_back(cpmsg);
                }
                for (auto [cp_name, cp_value] : result.performance_values)
                {
                    adore_if_ros_msg::CostPair cpmsg;
                    cpmsg.objective_name = cp_name;
                    cpmsg.objective_value = cp_value;
                    msg.performance_values.push_back(cpmsg);
                }
                // copy the nominal_maneuver_swath
                msg.nominal_maneuver_swath.trackingID = 0;
                msg.nominal_maneuver_swath.branchID = 0;
                msg.nominal_maneuver_swath.predecessorID = 0;
                msg.nominal_maneuver_swath.confidence = 0;
                for (const auto& pair : result.nominal_maneuver_swath.getLevel(0))
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
                // copy the combined_maneuver_swath
                msg.combined_maneuver_swath.trackingID = 0;
                msg.combined_maneuver_swath.branchID = 0;
                msg.combined_maneuver_swath.predecessorID = 0;
                msg.combined_maneuver_swath.confidence = 0;
                for (const auto& pair : result.combined_maneuver_swath.getLevel(0))
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
                // copy details on longitudinal planning
                // plan
                auto& longitudinal_plan = result.nominal_maneuver_longitudinal_plan.getData();
                const int NC_lon = longitudinal_plan.nc();
                const int NR_lon = longitudinal_plan.nr();
                if (NC_lon > 0 && NR_lon > 1)
                {
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_longitudinal_plan.fcn.data.insert(
                        msg.nominal_maneuver_longitudinal_plan.fcn.data.end(), longitudinal_plan.begin(),
                        longitudinal_plan.end());
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].size = NC_lon;
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].stride = NR_lon * NC_lon;
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[1].label = "s and derivatives";
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[1].size = (NR_lon - 1) * NC_lon;
                    msg.nominal_maneuver_longitudinal_plan.fcn.layout.dim[1].stride = NC_lon;
                }
                // upper bound
                auto& longitudinal_ubx = result.nominal_maneuver_longitudinal_ubx.getData();
                const int NC_lon_ubx = longitudinal_ubx.nc();
                const int NR_lon_ubx = longitudinal_ubx.nr();
                if (NC_lon_ubx > 0 && NR_lon_ubx > 1)
                {
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.data.insert(
                        msg.nominal_maneuver_longitudinal_plan_ubx.fcn.data.end(), longitudinal_ubx.begin(),
                        longitudinal_ubx.end());
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim.push_back(
                        std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].size = NC_lon_ubx;
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].stride = NR_lon_ubx * NC_lon_ubx;
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim.push_back(
                        std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[1].label = "upper bound of s and "
                                                                                         "derivatives";
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[1].size = (NR_lon_ubx - 1) * NC_lon_ubx;
                    msg.nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[1].stride = NC_lon_ubx;
                }
                // lower bound
                auto& longitudinal_lbx = result.nominal_maneuver_longitudinal_lbx.getData();
                const int NC_lon_lbx = longitudinal_lbx.nc();
                const int NR_lon_lbx = longitudinal_lbx.nr();
                if (NC_lon_lbx > 0 && NR_lon_lbx > 1)
                {
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.data.insert(
                        msg.nominal_maneuver_longitudinal_plan_lbx.fcn.data.end(), longitudinal_lbx.begin(),
                        longitudinal_lbx.end());
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim.push_back(
                        std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].size = NC_lon_lbx;
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].stride = NR_lon_lbx * NC_lon_lbx;
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim.push_back(
                        std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[1].label = "lowr bound of s and "
                                                                                         "derivatives";
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[1].size = (NR_lon_lbx - 1) * NC_lon_lbx;
                    msg.nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[1].stride = NC_lon_lbx;
                }
                // copy details on lateral planning
                // plan
                auto& lateral_plan = result.nominal_maneuver_lateral_plan.getData();
                const int NC_lat = lateral_plan.nc();
                const int NR_lat = lateral_plan.nr();
                if (NC_lat > 0 && NR_lat > 1)
                {
                    msg.nominal_maneuver_lateral_plan.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_lateral_plan.fcn.data.insert(msg.nominal_maneuver_lateral_plan.fcn.data.end(),
                                                                      lateral_plan.begin(), lateral_plan.end());
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[0].size = NC_lat;
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[0].stride = NR_lat * NC_lat;
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[1].label = "n and derivatives";
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[1].size = (NR_lat - 1) * NC_lat;
                    msg.nominal_maneuver_lateral_plan.fcn.layout.dim[1].stride = NC_lat;
                }
                // upper bound
                auto& lateral_ubx = result.nominal_maneuver_lateral_ubx.getData();
                const int NC_lat_ubx = lateral_ubx.nc();
                const int NR_lat_ubx = lateral_ubx.nr();
                if (NC_lat_ubx > 0 && NR_lat_ubx > 1)
                {
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.data.insert(
                        msg.nominal_maneuver_lateral_plan_ubx.fcn.data.end(), lateral_ubx.begin(), lateral_ubx.end());
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].size = NC_lat_ubx;
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].stride = NR_lat_ubx * NC_lat_ubx;
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[1].label = "upper bound of n and derivatives";
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[1].size = (NR_lat_ubx - 1) * NC_lat_ubx;
                    msg.nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[1].stride = NC_lat_ubx;
                }
                // lower bound
                auto& lateral_lbx = result.nominal_maneuver_lateral_lbx.getData();
                const int NC_lat_lbx = lateral_lbx.nc();
                const int NR_lat_lbx = lateral_lbx.nr();
                if (NC_lat_lbx > 0 && NR_lat_lbx > 1)
                {
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.data_offset = 0;
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.data.insert(
                        msg.nominal_maneuver_lateral_plan_lbx.fcn.data.end(), lateral_lbx.begin(), lateral_lbx.end());
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].label = "time";
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].size = NC_lat_lbx;
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].stride = NR_lat_lbx * NC_lat_lbx;
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim.push_back(std_msgs::MultiArrayDimension());
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[1].label = "lower bound of n and derivatives";
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[1].size = (NR_lat_lbx - 1) * NC_lat_lbx;
                    msg.nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[1].stride = NC_lat_lbx;
                }
                return msg;
            }
            /**
             * Conversion of PlanningResult message to PlanningResult
             */
            template <typename Tmsg>
            void operator()(Tmsg msg, adore::fun::PlanningResult& result)
            {
                result.id = msg->id;
                result.iteration = msg->iteration;
                result.name = msg->name;
                spr_converter_(&msg->nominal_maneuver, &result.nominal_maneuver);
                spr_converter_(&msg->combined_maneuver, &result.combined_maneuver);
                tr_converter_(&msg->terminal_maneuver, &result.terminal_maneuver);
                result.nominal_maneuver_valid = msg->nominal_maneuver_valid;
                result.combined_maneuver_valid = msg->combined_maneuver_valid;
                result.status_string = msg->status_string;
                result.maneuver_type = msg->maneuver_type;
                result.indicator_left = msg->indicator_left;
                result.indicator_right = msg->indicator_right;
                for (auto [name, value] : msg->objective_values)
                {
                    result.objective_values.insert({ name, value });
                }
                for (auto [name, value] : msg->performance_values)
                {
                    result.performance_values.insert({ name, value });
                }
                // copy nominal_maneuver_swath
                for (auto& msgi : msg->nominal_maneuver_swath.occupancy)
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
                // copy combined_maneuver_swath
                for (auto& msgi : msg->combined_maneuver_swath.occupancy)
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

                // copy details on longitudinal planning
                if (!msg->nominal_maneuver_longitudinal_plan.fcn.data.empty() &&
                    !msg->nominal_maneuver_longitudinal_plan.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_longitudinal_plan.getData() =
                        dlib::mat(&*msg->nominal_maneuver_longitudinal_plan.fcn.data.begin(),
                                  msg->nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_longitudinal_plan.fcn.layout.dim[0].size);
                }
                if (!msg->nominal_maneuver_longitudinal_plan_lbx.fcn.data.empty() &&
                    !msg->nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_longitudinal_lbx.getData() =
                        dlib::mat(&*msg->nominal_maneuver_longitudinal_plan_lbx.fcn.data.begin(),
                                  msg->nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_longitudinal_plan_lbx.fcn.layout.dim[0].size);
                }
                if (!msg->nominal_maneuver_longitudinal_plan_ubx.fcn.data.empty() &&
                    !msg->nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_longitudinal_ubx.getData() =
                        dlib::mat(&*msg->nominal_maneuver_longitudinal_plan_ubx.fcn.data.begin(),
                                  msg->nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_longitudinal_plan_ubx.fcn.layout.dim[0].size);
                }
                // copy details on lateral planning
                if (!msg->nominal_maneuver_lateral_plan.fcn.data.empty() &&
                    !msg->nominal_maneuver_lateral_plan.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_lateral_plan.getData() =
                        dlib::mat(&*msg->nominal_maneuver_lateral_plan.fcn.data.begin(),
                                  msg->nominal_maneuver_lateral_plan.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_lateral_plan.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_lateral_plan.fcn.layout.dim[0].size);
                }
                if (!msg->nominal_maneuver_lateral_plan_lbx.fcn.data.empty() &&
                    !msg->nominal_maneuver_lateral_plan_lbx.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_lateral_lbx.getData() =
                        dlib::mat(&*msg->nominal_maneuver_lateral_plan_lbx.fcn.data.begin(),
                                  msg->nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_lateral_plan_lbx.fcn.layout.dim[0].size);
                }
                if (!msg->nominal_maneuver_lateral_plan_ubx.fcn.data.empty() &&
                    !msg->nominal_maneuver_lateral_plan_ubx.fcn.layout.dim.empty())
                {
                    result.nominal_maneuver_lateral_ubx.getData() =
                        dlib::mat(&*msg->nominal_maneuver_lateral_plan_ubx.fcn.data.begin(),
                                  msg->nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].stride /
                                      msg->nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].size,
                                  msg->nominal_maneuver_lateral_plan_ubx.fcn.layout.dim[0].size);
                }
            }
        };
    }  // namespace if_ROS
}  // namespace adore
