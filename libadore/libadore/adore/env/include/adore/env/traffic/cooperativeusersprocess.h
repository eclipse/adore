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
#include <adore/mad/arraymatrixtools.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/logic/platoonLogic.h>
#include <adore/mad/cubicpiecewisefunction.h>
#include <adore/fun/nlpqlp_planner/colors.h>
namespace adore
{
    namespace env
    {
            
            class CooperativeUsersProcess
            {
                private:
                adore::params::APVehicle* pveh;
                ///prediction plus CACC(Platoon)
                struct CooperativeUser
                {
                    CooperativeUserPrediction MCM;
                    int PlatooningState = logic::NotAble;
                    bool inPlatooningRange = false;
                    double progress_front_side;
                    double progress_rear_side;
                };
                typedef std::vector<CooperativeUser> ListOfCooperativeUsers;


                ListOfCooperativeUsers Cooperative_frontMans;
                ListOfCooperativeUsers Cooperative_behindMans;
                CooperativeUser preceding; 
                CooperativeUser leader; 
                CooperativeUser following; 
                bool preceding_exist, leader_exist, precedingIsLeader, following_exist;
                bool frontManIsCooperative, followingIsCooperative;
                double ego_progress_front_side;
                double ego_progress_rear_side;
                adore::env::ThreeLaneViewDecoupled* three_lanes_;
                double preceding_dist_to_ego;
                double following_dist_to_ego;
                double leader_dist_to_ego;
                double leader_tau, preceding_tau, following_tau;
                adore::mad::CubicPiecewiseFunction::PieceweisePolynomial pp_v;
                double possiblePlatooningTimeHeadwayRange;
                double platooningDistance;
                int DEBUG_LEVEL;

                public:
                
                ListOfCooperativeUsers getFrontMansList() {return Cooperative_frontMans;}
                ListOfCooperativeUsers getBehindMansList() {return Cooperative_behindMans;}
                CooperativeUser getPreceding() {return preceding;}
                CooperativeUser getLeader() {return leader;}
                CooperativeUser getFollowing() {return following;}
                bool precedingExists() {return preceding_exist;}
                bool leaderExists() {return leader_exist;}
                bool followingExists() {return following_exist;}
                bool leaderIsPreceding() {return precedingIsLeader;}
                double getDistanceToLeader() {return leader_dist_to_ego;}
                double getDistanceToPreceding() {return preceding_dist_to_ego;}
                double getDistanceToFollowing() {return following_dist_to_ego;}
                double getEgoProgressFrontSide() {return ego_progress_front_side;}
                double getEgoProgressRearSide() {return ego_progress_rear_side;}
                double getLeaderTimeHeadway() {return leader_tau;}
                double getPrecedingTimeHeadway() {return preceding_tau;}
                double getFollowingTimeHeadway() {return following_tau;}
                bool noVehicleBetweenEgoAndPreceding() {return frontManIsCooperative;}
                bool noVehicleBetweenEgoAndFollowing() {return followingIsCooperative;}
                CooperativeUsersProcess(adore::env::ThreeLaneViewDecoupled* three_lanes_, adore::params::APVehicle* pveh, int DEBUG_LEVEL = 0,double possiblePlatooningTimeHeadwayRange = 6.)
                {
                    this->three_lanes_ = three_lanes_;
                    this->pveh = pveh;
                    ego_progress_front_side = 0.;
                    ego_progress_rear_side = 0.;
                    this->possiblePlatooningTimeHeadwayRange = possiblePlatooningTimeHeadwayRange;
                    platooningDistance = 50.;
                    this->DEBUG_LEVEL = DEBUG_LEVEL;


                }
                ~CooperativeUsersProcess()
                {
                    delete [] three_lanes_;

                }
                void process(adore::fun::VehicleMotionState9d& egoStates, CooperativeUsersList& list,adore::view::TrafficObject& frontMan,adore::view::TrafficObject& behindMan)
                {
                    preceding_exist = false;
                    leader_exist = false;
                    following_exist = false;
                    precedingIsLeader = false;
                    sort(egoStates,list);
                    compare(frontMan,behindMan);



                }
                private:
                // void addAcceleration(CooperativeUser user)
                // {
                //     std::vector<double> w;
                //     for(int i=0; i<user.MCM.currentTrajectory.v.size(); i++) w.push_back(1.);
                //     double *v = new double [user.MCM.currentTrajectory.v.size()];
                //     double *a = new double [user.MCM.currentTrajectory.v.size()];
                //     adore::mad::CubicPiecewiseFunction::fit(&pp_v,  &user.MCM.currentTrajectory.t0[0],&user.MCM.currentTrajectory.v[0],&w[0],user.MCM.currentTrajectory.v.size(), 0.9);
                //     adore::mad::CubicPiecewiseFunction::CubicSplineEvaluation(&v[0],&a[0],&user.MCM.currentTrajectory.t0[0],user.MCM.currentTrajectory.t0.size(),pp_v);
                //     for(int i=0; i<user.MCM.currentTrajectory.v.size(); i++)
                //     {
                //         user.MCM.currentTrajectory.a.push_back(a[i]);
                //     }
                //     std::cout<<"\n"<<a[0]<<"\t"<<a[1];
                //     delete [] a;
                //     delete [] v;

                // }
                //check is cooperative preceding is the same as front man detected by ego sensor
                void compare(adore::view::TrafficObject& frontMan,adore::view::TrafficObject& behindMan)
                {
                    //preceding_progress = 0.;
                    double marginTau = 1.;
                    frontManIsCooperative = false;
                    followingIsCooperative = false;
                    double s_fr = frontMan.getCurrentProgress();
                    double s_bm = behindMan.getCurrentProgress();
                   // std::cout<<"\n"<<s_fr <<"\t"<< preceding.progress_rear_side;
                   if(std::abs(s_fr - preceding.progress_rear_side)  < std::max(frontMan.getCurrentSpeed()*marginTau,5.))
                   {
                       frontManIsCooperative = true;
                   }
                  
                   if(std::abs(s_bm - following.progress_front_side)  < std::max(behindMan.getCurrentSpeed()*marginTau,5.))
                   {
                       followingIsCooperative = true;
                   }                   
                }


                void sort(adore::fun::VehicleMotionState9d& egoStates, CooperativeUsersList& list)
                {
                    Cooperative_behindMans.clear();
                    Cooperative_frontMans.clear();
                    preceding.MCM.clear();
                    leader.MCM.clear();
                    following.MCM.clear();
                    CooperativeUser tmp;
                    std::vector<double> s;
                    if(three_lanes_->getCurrentLane()->isValid() && list.size())
                    {
                        double ego_n, ego_s;
                        three_lanes_->getCurrentLane()->toRelativeCoordinates(egoStates.getX() ,egoStates.getY() ,ego_s,ego_n);
                        double x_ego_front_side, y_ego_front_side;
                        double x_ego_rear_side, y_ego_rear_side;
                        x_ego_front_side = egoStates.getX()  +  std::cos(egoStates.getPSI())*(pveh->get_a()+pveh->get_b()+pveh->get_c()) ;
                        y_ego_front_side = egoStates.getY()   +  std::sin(egoStates.getPSI())*(pveh->get_a()+pveh->get_b()+pveh->get_c()) ;
                        three_lanes_->getCurrentLane()->toRelativeCoordinates(x_ego_front_side,y_ego_front_side,ego_progress_front_side,ego_n);

                        x_ego_rear_side = egoStates.getX()  +  std::cos(egoStates.getPSI())*(-pveh->get_d()) ;
                        y_ego_rear_side = egoStates.getY()   +  std::sin(egoStates.getPSI())*(-pveh->get_d()) ;
                        three_lanes_->getCurrentLane()->toRelativeCoordinates(x_ego_front_side,y_ego_front_side,ego_progress_rear_side,ego_n);                        for(int i=0; i<list.size(); i++)
                       for(int i=0; i<list.size(); i++)
                       {
                            
                            //TO DO : check if they are in the same lane as ego
                            //TO DO: if data is still valid
                            
                            if(list[i].currentTrajectory.x.size() > 1) 
                            {
                                double _s, _n;
                                double x = list[i].currentTrajectory.x[0];
                                double y = list[i].currentTrajectory.y[0];
                                three_lanes_->getCurrentLane()->toRelativeCoordinates(x,y,_s,_n);
                                s.push_back(_s);
                            }
                        }
                        if(s.size())
                        {
                         double *s_sorted = new double [s.size()];
                         int *index = new int [s.size()];
                         adore::mad::ArrayMatrixTools::sort(&s_sorted[0],&index[0],&s[0],s.size());
                

                            for(int i=0; i<s.size(); i++)
                            {

                                if(s_sorted[i]<ego_s)
                                {
                                    tmp.MCM = list[index[i]];
                                    //addAcceleration(tmp);
                                    tmp.MCM.communicationDelay = egoStates.getTime() - tmp.MCM.currentTrajectory.t0[0];
                                    tmp.progress_front_side = s_sorted[i];  //MCM is based on front side
                                    double x, y, _n;
                                    x = tmp.MCM.currentTrajectory.x[0];
                                    y = tmp.MCM.currentTrajectory.y[0];  
                                    toCenterOfTheRearSide(x, y,tmp.MCM.currentTrajectory.psi[0],tmp.MCM.vehicleLength );
                                    //processIntention(&tmp);
                                    three_lanes_->getCurrentLane()->toRelativeCoordinates(x,y,tmp.progress_rear_side ,_n);
                                    
                                    Cooperative_behindMans.push_back(tmp);  
                                    tmp.MCM.clear();
                                    //std::cout<<"\nbehindMans: "<<Cooperative_behindMans.back().MCM.id<<"\t"<<Cooperative_behindMans.back().MCM.currentTrajectory.x[0]<<"\t"<<Cooperative_behindMans.back().MCM.currentTrajectory.y[0];

                                }
                                else
                                {
 
                                    tmp.MCM = list[index[i]];
                                   // addAcceleration(tmp);
                                    tmp.MCM.communicationDelay = egoStates.getTime() - tmp.MCM.currentTrajectory.t0[0];
                                    tmp.progress_front_side = s_sorted[i];  //MCM is based on front side
                                    double x, y, _n;
                                    x = tmp.MCM.currentTrajectory.x[0];
                                    y = tmp.MCM.currentTrajectory.y[0]; 
        
                                    //std::cout<<"\n"<<tmp.MCM.currentTrajectory.psi[0]<<"\t"<<tmp.MCM.vehicleLength ;
    
                                    toCenterOfTheRearSide(x, y,tmp.MCM.currentTrajectory.psi[0],tmp.MCM.vehicleLength );
                                
                                    //processIntention(&tmp);
                                    three_lanes_->getCurrentLane()->toRelativeCoordinates(x,y,tmp.progress_rear_side ,_n);
                                                                    
                                    //processIntention(&tmp);
                                    Cooperative_frontMans.push_back(tmp);
                                    //std::cout<<"\nfrontMans: "<<Cooperative_frontMans.back().MCM.id<<"\t"<<Cooperative_frontMans.back().MCM.currentTrajectory.x[0]<<"\t"<<Cooperative_frontMans.back().MCM.currentTrajectory.y[0];

                                }
        
                            }
                            delete [] s_sorted;
                            delete [] index;
                            if(Cooperative_frontMans.size())
                            {
                                preceding_exist = true;
                                preceding = Cooperative_frontMans[0];
                                preceding_dist_to_ego = preceding.progress_rear_side - ego_progress_front_side;
                                preceding_tau = (preceding_dist_to_ego)/std::max(egoStates.getvx(),0.10);
                                preceding.inPlatooningRange = false;
                                if(preceding_tau<= possiblePlatooningTimeHeadwayRange || preceding_dist_to_ego<platooningDistance) preceding.inPlatooningRange = true;
                               processIntention(&preceding);
                                if(Cooperative_frontMans.size()==1)
                                {
                                    leader = preceding;
                                    leader_exist = true;
                                    precedingIsLeader = true;
                                    leader_dist_to_ego = preceding_dist_to_ego;
                                    leader_tau = preceding_tau;
                                    
                                    


                                }
                                else
                                {
                                    
                                    leader = Cooperative_frontMans.back();
                                    leader_exist = true;       
                                    leader_dist_to_ego = leader.progress_rear_side - ego_progress_front_side;  
                                    leader_tau = (leader_dist_to_ego)/std::max(egoStates.getvx(),0.10); 
                                    leader.inPlatooningRange = false;
                                    processIntention(&leader);
                                    if(leader_tau<= possiblePlatooningTimeHeadwayRange || leader_dist_to_ego<platooningDistance) leader.inPlatooningRange = true;                                                    
                                }
                            }
                            if(Cooperative_behindMans.size())
                            {
                                following_exist = true;
                                following = Cooperative_behindMans.back();
                                following_dist_to_ego = ego_progress_rear_side - following.progress_front_side ;
                                following_tau = (following_dist_to_ego)/std::max(following.MCM.currentTrajectory.v[0],0.10);
                                following.inPlatooningRange = false;
                                if(following_tau<= possiblePlatooningTimeHeadwayRange || following_dist_to_ego<platooningDistance) following.inPlatooningRange = true;
                                processIntention(&following);
                            }
                        }                      

                    } 
                    //std::cout<<"\n-*-*-*-*-*-*LANE IS NOT VALID";                 

                }
                void processIntention(CooperativeUser* cu)
                {
                    print_debug(cu);
                    double tieBreaker = 0.5;
                    double safeTau = 3.0 - tieBreaker; //safe time headway
                    cu->PlatooningState = logic::NotAble;  //initialization
                    if(cu->MCM.currentTrajectory.x.size() == 0)  //No trajectory included
                    {
                        cu->PlatooningState = logic::NotAble;
                        // std::cout<<"\n"<<cu->MCM.id<<"------------------------NOT ABLE";
                    }
                    else
                    {
                        double safeDistance = safeTau*std::max(cu->MCM.currentTrajectory.v[0],3.);

                        if(cu->MCM.target_automation_level ==  adore::fun::logic::SAE_LEVEL3 &&
                           cu->MCM.toletated_distance_ahead <= (safeDistance) &&
                           cu->MCM.toletated_distance_behind <= (safeDistance))
                        {
                            cu->PlatooningState = logic::WantToForm;
                        //    std::cout<<"\n"<<cu->MCM.id<<"------------------------WANT TO FORM";


                        }
                        if(cu->MCM.target_automation_level ==  adore::fun::logic::SAE_LEVEL3 &&
                           cu->MCM.toletated_distance_ahead > (safeDistance) &&
                           cu->MCM.toletated_distance_behind > (safeDistance))
                        {
                            std::cout<<"\n"<<cu->MCM.toletated_distance_ahead<<"\t"<<cu->MCM.toletated_distance_behind <<"\t"<<safeDistance<<"\t"<<cu->MCM.currentTrajectory.v[0];
                            cu->PlatooningState = logic::Leaving;
                        //    std::cout<<"\n"<<cu->MCM.id<<"------------------------WANT TO FORM";


                        }                        
                    }

                }
                void intentionPredictionPrint(CooperativeUser* cu)
                {
                     if(cu->MCM.currentTrajectory.x.size() == 0)  BOLD(FRED("MCM.NoTrajectory"));
                     std::cout<<"\n"<< BOLD(FRED("MCM.toletated_distance_ahead: "))<<cu->MCM.toletated_distance_ahead ;
                     std::cout<<"\n"<<BOLD(FRED("MCM.toletated_distance_behind: "))<<cu->MCM.toletated_distance_behind ;
                     std::cout<<"\n"<<BOLD(FRED("MCM.target_automation_level: "))<<unsigned(cu->MCM.target_automation_level) ;
                     //printf("\n*****%i",(cu->MCM.target_automation_level) );

                }

                void print_debug(CooperativeUser* cu)
                {
                    //std::cout<<"\n"<<DEBUG_LEVEL;
                    if(DEBUG_LEVEL)
                    {
                        intentionPredictionPrint(cu);

                    }

                }
                void toCenterOfTheRearSide(double &x, double &y, double psi, double vehicleLength)
                {
                x = x  +  std::cos(psi)*(-vehicleLength) ;
                y = y  +  std::sin(psi)*(-vehicleLength) ;
                }


            };
    }
}