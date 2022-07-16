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
#include <adore/fun/logic/platoonStateMachine.h>
#include <adore/fun/logic/distanceStateMachine.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/cooperativeusersprocess.h>
#include <adore/fun/nlpqlp_planner/colors.h>
#include <adore/fun/nlpqlp_planner/planner_data.h>
//#include <adore/view/platoonview.h>

namespace adore
{
        namespace fun
    {
        using EnvFactory = adore::env::EnvFactoryInstance;
        using ParamsFactory = adore::params::ParamsFactoryInstance;
        using FunFactory = adore::fun::FunFactoryInstance;   
        using namespace adore::env;  
        using namespace adore::fun::logic;   
        class platoonLogic
        {
            
            public:
            int CAR_FOLLOWING_TACTICS;
            enum CAR_FOLLOWING_TACTICS {IDM, CACC};
            adore::env::BorderBased::LocalRoadMap roadmap_;/**< roadmap is used for precedence rules*/
            adore::mad::AWriter<adore::fun::PlatooningInformation>* platooningInformation_writer;
            adore::fun::PlatooningInformation platooningInformation;  
            
            enum LanePosition {offTheRoad = -1, hardShoulder=0, outermostDrivingLane = 1, secondLaneFromOutside = 2, thirdLaneFromOutside = 3, fourthLaneFromOutside = 4, fifthLaneFromOutside = 5 };
            
            adore::fun::logic::PlatooningStateMachine psm; 
            adore::fun::logic::DistanceStateMachine dsm; 
            int v2xStationID;
            int lanePosition;
            double communicationDelayThreshold;
            Planner_data data;
           // adore::view::platoonView* p_view;          
            
            platoonLogic(adore::env::ThreeLaneViewDecoupled* three_lanes_,adore::env::CooperativeUsersProcess* CooperativeUsersProcess,  Planner_data data): roadmap_(EnvFactory::get(),ParamsFactory::get())
            { 
                this->data = data;
                platooningInformation_writer =  FunFactory::get()->getPlatooningStateWriter();
                this->three_lanes_ = three_lanes_;  
                this->v2xStationID =  data.v2xStationID ;
                this->CooperativeUsersProcess = CooperativeUsersProcess;
                possiblePlatooningTau = 5.;
                eps = 2.0;
                communicationDelayThreshold = 0.5;
                CAR_FOLLOWING_TACTICS = IDM;
                
                //
                
                //p_view = new adore::view::platoonView(three_lanes_);             
                psm.initiate();
                dsm.initiate();
                
                
                     
            }
            ~platoonLogic()
            {
                delete three_lanes_;
            }
            int update(adore::fun::VehicleMotionState9d& egoStates, int lanePosition=1)
            {
                CAR_FOLLOWING_TACTICS = IDM;
                NoCommunicationDelay_p = false;
                sameLane_p = false;
                


                 this->lanePosition = lanePosition;
                 if(CooperativeUsersProcess->precedingExists())
                 {
                    
                    auto preceding = CooperativeUsersProcess->getPreceding();
                    if(preceding.MCM.communicationDelay < communicationDelayThreshold) NoCommunicationDelay_p= true;
                    if(preceding.MCM.lane_position == lanePosition ) sameLane_p = true;

                    //transition  [want to form --> joining]                               
                    if((preceding.PlatooningState == logic::WantToForm ||preceding.PlatooningState == logic::InPlatoon) &&
                    (psm.getPlatooningState() == logic::WantToForm || psm.getPlatooningState() == logic::InPlatoon) &&
                    CooperativeUsersProcess->noVehicleBetweenEgoAndPreceding() &&
                    sameLane_p &&
                    preceding.inPlatooningRange &&
                    NoCommunicationDelay_p)
                    {
                        //std::cout<<"\nWF2J";
                        psm.process_event(WF2J());
                        
                       // psm.process_event(IP2J());
                         //transition  [joining --> in platoon] 
                         //std::cout<<"\n"<<CooperativeUsersProcess->getPrecedingTimeHeadway()<<"\t"<<psm.getInPlatoonTimeHeadway();
    

                    } 
                    if((preceding.PlatooningState == logic::WantToForm ||preceding.PlatooningState == logic::InPlatoon) &&
                    (psm.getPlatooningState() == logic::Joining) &&
                    CooperativeUsersProcess->noVehicleBetweenEgoAndPreceding() &&
                    sameLane_p &&
                    preceding.inPlatooningRange &&
                    NoCommunicationDelay_p &&
                    CooperativeUsersProcess->getPrecedingTimeHeadway() < psm.getInPlatoonTimeHeadway()+eps)
                    {      
                        //std::cout<<"\nJ2IP";
                        psm.process_event(J2IP());

                    }
                    //[(joining || inplatoon) --> want to form]            

                    if( ((preceding.PlatooningState == logic::Leaving ||preceding.PlatooningState == logic::NotAble) &&
                    (psm.getPlatooningState() == logic::WantToForm || psm.getPlatooningState() == logic::InPlatoon || psm.getPlatooningState() == logic::Joining)) ||
                    !CooperativeUsersProcess->noVehicleBetweenEgoAndPreceding() ||
                    !sameLane_p ||
                    !NoCommunicationDelay_p)
                    {
                        //std::cout<<"\nJ2WF & IP2WF";
                        psm.process_event(J2WF());
                        psm.process_event(IP2WF());


                    }
                 }
                 
                 //I am the leading vehicle, no preceding exists
                if(CooperativeUsersProcess->precedingExists() == false && CooperativeUsersProcess->leaderExists() == false)
                {
                    if(CooperativeUsersProcess->followingExists())
                    {
                        auto following = CooperativeUsersProcess->getFollowing();
                        //std::cout<<"\n"<<CooperativeUsersProcess->getDistanceToFollowing()<<"\t"<< CooperativeUsersProcess->getFollowingTimeHeadway() ;
                        if( (following.PlatooningState == logic::WantToForm || following.PlatooningState == logic::Joining || following.PlatooningState == logic::InPlatoon ) &&
                          (psm.getPlatooningState() == logic::WantToForm || psm.getPlatooningState() == logic::InPlatoon) &&
                           following.MCM.lane_position == lanePosition &&
                           CooperativeUsersProcess->getFollowingTimeHeadway() < psm.getInPlatoonTimeHeadway()+eps)
                           {
                               psm.process_event(WF2IP());
                               psm.process_event(J2IP());

                           }
                    if( ((following.PlatooningState == logic::Leaving ||following.PlatooningState == logic::NotAble) &&
                    (psm.getPlatooningState() == logic::WantToForm || psm.getPlatooningState() == logic::InPlatoon || psm.getPlatooningState() == logic::Joining)) ||
                    !CooperativeUsersProcess->noVehicleBetweenEgoAndFollowing() ||
                    following.MCM.lane_position != lanePosition ||
                    following.MCM.communicationDelay > communicationDelayThreshold)
                    {
                        psm.process_event(J2WF());
                        psm.process_event(IP2WF());


                    }                           

                    }

                }

           
                
                

                
                // psm.process_event(adore::fun::logic::WF2J());
                // psm.process_event(adore::fun::logic::J2NA());
              
                
                // int a; std::cin>>a;
                // //psm.process_event()
                double platooningBasedDistance = std::max(egoStates.getvx(),3.)*psm.getTimeHeadway();
                platooningInformation.setId(v2xStationID);
                platooningInformation.setLanePosition(lanePosition);  //TODO : must be calculated
                platooningInformation.setTargetAutomationLevel(logic::SAE_LEVEL3);
                platooningInformation.setToleratedDistanceAhead(platooningBasedDistance);//TODO : must be calculated
                platooningInformation.setToleratedDistanceBehind(platooningBasedDistance);//TODO : must be calculated
                // std::cout<<"\n"<<platooningInformation.getId();
                // std::cout<<"\n"<<platooningInformation.getLanePosition();
                // std::cout<<"\n"<<platooningInformation.getTargetAutomationLevel();
                platooningInformation_writer->write(platooningInformation);
                if(data.print_strategy) print();
                if(psm.getPlatooningState() == logic::Joining || psm.getPlatooningState() == logic::InPlatoon) CAR_FOLLOWING_TACTICS = CACC;
                //because I am leading, I am in platoon with following and not (yet) with preceding
                if(CooperativeUsersProcess->precedingExists() == false && CooperativeUsersProcess->leaderExists() == false && psm.getPlatooningState() == logic::InPlatoon) CAR_FOLLOWING_TACTICS = IDM;
                {

                }
                return CAR_FOLLOWING_TACTICS;
          

            }
            //adore::fun::VehicleMotionState9d &egoStates
            private:
            bool NoCommunicationDelay_p;
            bool sameLane_p;
            void print()
            {
                std::cout<<"\n--------------------------------";

                std::cout<<"\nEgo is:    "<<v2xStationID;
                printPlatooningState( psm.getPlatooningState() );
                //std::cout<<"\n"
                if(CooperativeUsersProcess->precedingExists())
                 {
                     auto preceding = CooperativeUsersProcess->getPreceding();
                     std::cout<<"\nPreceding: "<<preceding.MCM.id;
                     printPlatooningState( preceding.PlatooningState );
                     
                     if(preceding.MCM.lane_position== lanePosition)  std::cout<<"  SAME LANE";
                     if(preceding.inPlatooningRange)  std::cout<<"  IN RANGE";
                     if(preceding.MCM.communicationDelay>communicationDelayThreshold)  std::cout<< BOLD(FRED("  DELAY"));
                     if(!CooperativeUsersProcess->noVehicleBetweenEgoAndPreceding())  std::cout<< BOLD(FRED("  SomeoneELse"));
                     //std::cout<<"\n"<<CooperativeUsersProcess->getDistanceToPreceding()<<"\t"<<CooperativeUsersProcess->getPrecedingTimeHeadway();
                 }
                if(CooperativeUsersProcess->followingExists())
                 {
                     auto following = CooperativeUsersProcess->getFollowing();
                     std::cout<<"\nFollowing: "<<following.MCM.id;
                     printPlatooningState( following.PlatooningState );
                     
                     if(following.MCM.lane_position== lanePosition)  std::cout<<"  SAME LANE";
                     if(following.inPlatooningRange)  std::cout<<"  IN RANGE";
                     if(following.MCM.communicationDelay>communicationDelayThreshold)  std::cout<< BOLD(FRED("  DELAY"));
                     if(!CooperativeUsersProcess->noVehicleBetweenEgoAndFollowing())  std::cout<< BOLD(FRED("  SomeoneELse"));
                     //std::cout<<"\n"<<CooperativeUsersProcess->getDistanceToPreceding()<<"\t"<<CooperativeUsersProcess->getPrecedingTimeHeadway();
                 }                 

                
            }
            void printPlatooningState(int ps)
            {
                if(ps == 0)  std::cout <<"\t"<< BOLD(FRED("NOT ABLE"));
                if(ps== 1)   std::cout <<"\t"<< BOLD(FYEL("WANT TO FORM"));
                if(ps == 2)  std::cout <<"\t"<< BOLD(FMAG("JOINING"));
                if(ps == 3)  std::cout <<"\t"<< BOLD(FGRN("IN PLATOON"));
                if(ps== 4)   std::cout <<"\t"<< BOLD(FRED("LEAVING"));
            }
            ///if the time headway to the preceding vehicle is less than this value we consider it for platooning
            double possiblePlatooningTau;
            adore::env::ThreeLaneViewDecoupled* three_lanes_;
            adore::env::CooperativeUsersProcess* CooperativeUsersProcess;
            double eps;
            
        };
    }
}