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
 *   Reza Dariani - computation of platoon view in separate app
 ********************************************************************************/

#pragma once

#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/afactory.h>
#include <adore/env/borderbased/lanefollowinggeometry.h>
#include <adore/env/borderbased/lanechangegeometry.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/env/tcd/connectionsonlane.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/mad/arraymatrixtools.h>
#include <vector>


namespace adore
{
    namespace apps
    {
        /**
         * @brief 
         *
         */
        class PlatoonViewProvider
        {
          private:

            adore::mad::AReader<adore::env::CooperativeUserPrediction>* cooperativeUserPrediction_reader;
            adore::mad::AWriter<adore::env::CooperativeUsersList>* cooperativeUserList_writer;
            adore::env::CooperativeUserPrediction cooperativeUser;
            adore::env::CooperativeUsersList localCooperativeUserList; //to buffer
            
            //std::vector<adore::env::CooperativeUserPrediction> cooperativeUserPredictionList;
            adore::mad::AReader<adore::fun::VehicleMotionState9d>* xreader_;
            adore::fun::VehicleMotionState9d x_;
            adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
            adore::env::ConnectionsOnLane* connectionsOnLane_;
            adore::env::ControlledConnectionSet connectionSet_;/**< current rule set for controlled connections*/
            std::vector<int> index;
            
                   


          public:
            PlatoonViewProvider() :connectionSet_(adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed() )
            {
        
                cooperativeUserPrediction_reader = adore::env::EnvFactoryInstance::get()->getCooperativeUserReader();
                cooperativeUserList_writer = adore::env::EnvFactoryInstance::get()->getCooperativeUsersListWriter();
                xreader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
                 connectionsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&connectionSet_); 
                 
                
            }
            virtual ~PlatoonViewProvider()
            {
                delete xreader_;
            }

            
            /**
             * @brief update data, views and recompute maneuver
             *
             */
            void run()
            {

                readCooperativeUser();
                cooperativeUserList_writer->write(localCooperativeUserList);
            }
          

            private:
            
            
            
            bool vehiclePlatooningCapability;
            int  automationLevel;
      
            int getIndex(std::vector<int> v, int K)
            {
                auto it = find(v.begin(), v.end(), K);
                if (it != v.end())
                {
                
                    return (it - v.begin());
                }
                else {return -1; }
            } 
                             
            void readCooperativeUser()
            {
            int ind = -1;
            //localCooperativeUserList.clear();
            cooperativeUser.clear();
            

               if(cooperativeUserPrediction_reader!=0 && cooperativeUserPrediction_reader->hasData())
                {                
                    cooperativeUserPrediction_reader->getData(cooperativeUser);                      
    
                        if(cooperativeUser.lane_position >= -1 && cooperativeUser.lane_position<=14)
                        {
             
                            if(localCooperativeUserList.empty() )
                             {
                             
                                 localCooperativeUserList.push_back(cooperativeUser);
                                 index.push_back(cooperativeUser.id);                       
                            
                             }
                            else
                            {
                                
                                ind = getIndex(index,  cooperativeUser.id);
                                if(ind == -1)
                                {
                          
                                    localCooperativeUserList.push_back(cooperativeUser);
                                    index.push_back(cooperativeUser.id);
                                }
                                else
                                {
                                  
                                localCooperativeUserList[ind] =   cooperativeUser; 
                                }

                            }

                        }
                        // 
                    
                }
                std::cout<<"\n"<<localCooperativeUserList.size()<<" cooperative object[s] is[are] listed";  
                for(int i=0; i<localCooperativeUserList.size(); i++) std::cout<<"\nID: "<<localCooperativeUserList[i].id; 
                // 
                
                //localCooperativeUserList                  
            }


      
        };
    }  // namespace apps
}  // namespace adore
