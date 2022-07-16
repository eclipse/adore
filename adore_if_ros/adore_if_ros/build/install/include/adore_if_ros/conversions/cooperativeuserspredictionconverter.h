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
 *   Reza Dariani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/env/afactory.h>
#include <adore_if_ros_msg/CooperativePlanningSet.h>
#include <adore/env/traffic/cooperativeusersprediction.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Conversions between adore::env::CooperativeUserPrediction and ROS messages.
         */
        struct CooperativeUsersPredictionConverter
        {
            public:

            template<typename Tmsg>//adore_if_ros_msg::CooperativePlanningSet 
            void operator()(Tmsg msg, adore::env::CooperativeUsersList * list)
            {
                list->clear();
                adore::env::CooperativeUserPrediction cup;
                for(int i=0; i<msg->data.size(); i++)
                {
                    cup.setId(msg->data[i].id);
                    cup.setLanePosition(msg->data[i].lane_position);
                    cup.setTargetAutomationLevel(msg->data[i].target_automation_level);
                    cup.setToletatedDistanceAhead(msg->data[i].tolerated_distance_ahead);
                    cup.setToletatedDistanceBehind(msg->data[i].tolerated_distance_behind);
                    cup.setVehicleLength (msg->data[i].vehicle_length);
                    cup.setVehicleWidth (msg->data[i].vehicle_width);
                    for(int j=0; j<msg->data[i].prediction.size(); j++)
                    {
                        cup.currentTrajectory.x.push_back(msg->data[i].prediction[j].x);
                        cup.currentTrajectory.y.push_back(msg->data[i].prediction[j].y);
                        cup.currentTrajectory.v.push_back(msg->data[i].prediction[j].v);
                        cup.currentTrajectory.psi.push_back(msg->data[i].prediction[j].psi);
                        cup.currentTrajectory.t0.push_back(msg->data[i].prediction[j].t0);
                        cup.currentTrajectory.t1.push_back(msg->data[i].prediction[j].t1);
                    } 
                    list->push_back(cup);   
                    cup.clear();  
                } 
                
            }
            template<typename Tmsg>//adore_if_ros_msg::CooperativePlanning
            void operator()(Tmsg msg, adore::env::CooperativeUserPrediction * user)
            {
                user->clear();
                user->setId(msg->id);
                user->setLanePosition(msg->lane_position);
                user->setTargetAutomationLevel(msg->target_automation_level);
                user->setToletatedDistanceAhead(msg->tolerated_distance_ahead);
                user->setToletatedDistanceBehind(msg->tolerated_distance_behind);
                user->setVehicleWidth(msg->vehicle_width);
                user->setVehicleLength(msg->vehicle_length);
                for(int i=0; i<msg->prediction.size(); i++)
                {
                    user->currentTrajectory.x.push_back(msg->prediction[i].x);
                    user->currentTrajectory.y.push_back(msg->prediction[i].y);
                    user->currentTrajectory.v.push_back(msg->prediction[i].v);
                    user->currentTrajectory.psi.push_back(msg->prediction[i].psi);
                    user->currentTrajectory.t0.push_back(msg->prediction[i].t0);
                    user->currentTrajectory.t1.push_back(msg->prediction[i].t1);


                }            
                
            }            

            adore_if_ros_msg::CooperativePlanningSet operator()(const adore::env::CooperativeUsersList& list)
            {
                 //std::cout<<"\nwrite p_info to msg";
                adore_if_ros_msg::CooperativePlanningSet msg;
                adore_if_ros_msg::CooperativePlanning cp;
                adore_if_ros_msg::CooperativeUser cu;
                for(int i=0; i<list.size(); i++)
                {
                     cp.id = list[i].id;
                     cp.lane_position = list[i].lane_position;
                     cp.target_automation_level = list[i].target_automation_level;
                     cp.tolerated_distance_ahead = list[i].toletated_distance_ahead;
                     cp.tolerated_distance_behind = list[i].toletated_distance_behind;
                     cp.vehicle_length = list[i].vehicleLength;
                     cp.vehicle_width = list[i].vehicleWidth;
                     for(int j=0; j<list[i].currentTrajectory.x.size(); j++)
                     {
                         cu.x = list[i].currentTrajectory.x[j];
                         cu.y = list[i].currentTrajectory.y[j];
                         cu.v = list[i].currentTrajectory.v[j];
                         cu.psi = list[i].currentTrajectory.psi[j];
                         cu.t0 = list[i].currentTrajectory.t0[j];
                         cu.t1 = list[i].currentTrajectory.t1[j];
                         cp.prediction.push_back(cu);
                     }                     
                     msg.data.push_back(cp);
                     cp.prediction.clear();
                }
                return msg;
            }

            adore_if_ros_msg::CooperativePlanning operator()(const adore::env::CooperativeUserPrediction& user)
            {
                adore_if_ros_msg::CooperativePlanning msg;

                msg.id = user.id;
                msg.lane_position = user.lane_position;
                msg.target_automation_level = user.target_automation_level;
                msg.tolerated_distance_ahead = user.toletated_distance_ahead;
                msg.tolerated_distance_behind = user.toletated_distance_behind;
                msg.vehicle_length = user.vehicleLength;
                msg.vehicle_width = user.vehicleWidth;
                adore_if_ros_msg::CooperativeUser cu;
                for(int i=0; i<user.currentTrajectory.x.size();i++)
                {
                    cu.x = user.currentTrajectory.x[i];
                    cu.y = user.currentTrajectory.y[i];
                    cu.v = user.currentTrajectory.v[i];
                    cu.psi = user.currentTrajectory.psi[i];
                    cu.t0 = user.currentTrajectory.t0[i];
                    cu.t1 = user.currentTrajectory.t1[i];
                    msg.prediction.push_back(cu);
                }
                return msg;





            }
            
        };




    }
}
