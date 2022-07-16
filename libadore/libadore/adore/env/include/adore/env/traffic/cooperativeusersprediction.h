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
#include <vector>
//

namespace adore
{
    namespace env
    {

            struct CooperativeUserPrediction
            {
                public:

                int id;
                double toletated_distance_behind;    //MCM based
                double toletated_distance_ahead;    //MCM based
                unsigned int target_automation_level;        //MCM based
                int lane_position;                  //MCM based
                double vehicleLength;               //MCM based
                double vehicleWidth;               //MCM based
                double communicationDelay;
                struct 
                {
                std::vector<double> x;              //MCM based
                std::vector<double> y;              //MCM based
                std::vector<double> v;              //MCM based
                std::vector<double> t0;             //MCM based
                std::vector<double> t1;             //MCM based
                std::vector<double> psi;             //MCM based
                std::vector<double> a; 
                } currentTrajectory;
                void setId(int id) {this->id = id;}
                void setLanePosition( int lane_position) {this->lane_position = lane_position;}
                void setTargetAutomationLevel(unsigned int target_automation_level) {this->target_automation_level = target_automation_level;}
                void setToletatedDistanceBehind(double toletated_distance_behind) {this->toletated_distance_behind = toletated_distance_behind;}
                void setToletatedDistanceAhead(double toletated_distance_ahead) {this->toletated_distance_ahead = toletated_distance_ahead;}
                void setVehicleLength(double vehicleLength) {this->vehicleLength = vehicleLength;}
                void setVehicleWidth(double vehicleWidth) {this->vehicleWidth = vehicleWidth;}
                void clear()
                {
                    currentTrajectory.x.clear();
                    currentTrajectory.y.clear();
                    currentTrajectory.v.clear();
                    currentTrajectory.t0.clear();
                    currentTrajectory.t1.clear();
                    currentTrajectory.a.clear();
                    currentTrajectory.psi.clear();

                }
            


                
                

            };
            typedef std::vector<CooperativeUserPrediction> CooperativeUsersList;

          

            
    }
}
