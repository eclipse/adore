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
 *    Daniel Heß - initial setup of prediction filter
 ********************************************************************************/
#pragma once
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
// #include <adore/fun/afactory.h>
#include <adore/env/traffic/occupancycylinderprediction.h>
#include <adore/env/traffic/ocroadbasedprediction.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/env/map/precedence_filter.h>
#include <adore/env/tcd/controlledconnection.h>
#include <adore/params/afactory.h>
#include <unordered_map>
#include <unordered_set>


namespace adore
{
    namespace apps
    {
        using EnvFactory = adore::env::EnvFactoryInstance;
        using ParamsFactory = adore::params::ParamsFactoryInstance;
        // using EnvFactory = adore::env::EnvFactoryInstance;
        class PredictionFilter
        {
            private:
                std::unordered_set<adore::env::ConnectionState::EConnectionState> blocking_states_;
                adore::env::AFactory::TOCPredictionSetReader* prediction_reader;/**< expected input predictions*/
                adore::env::AFactory::TOCPredictionSetWriter* prediction_writer;/**< expected output predictions*/
                adore::env::BorderBased::LocalRoadMap roadmap_;/**< roadmap is used for precedence rules*/
                adore::env::ThreeLaneViewDecoupled three_lanes_;/**< the lanes, for which to filter predictions*/
                adore::mad::AReader<adore::env::VehicleMotionState9d>* motion_state_reader_;/**< ego state used for filtering tail-gaters*/
                adore::env::ControlledConnectionSet connectionSet_;/**< current rule set for controlled connections*/
                adore::params::APLocalRoadMap* pLocalRoadMap_;/**< get discard radius parameter */
                adore::params::APVehicle* pVehicle_;/**< get dimensions of vehicle*/
                adore::params::APPrediction* pPrediction_;/**< get prediction options*/
                bool worstcase_;/**< whether filter is active for worstcase or for expected predictions*/
            public:
            PredictionFilter(bool worstcase) : 
                roadmap_(EnvFactory::get(),ParamsFactory::get()),three_lanes_(false/*do not monitor traffic*/),
                connectionSet_(EnvFactory::get()->getControlledConnectionFeed()),
                worstcase_(worstcase)
            {
                if(worstcase)
                {
                    prediction_reader = EnvFactory::get()->getWorstCaseRawPredictionSetReader();
                    prediction_writer = EnvFactory::get()->getWorstCasePredictionSetWriter();
                }
                else
                {
                    prediction_reader = EnvFactory::get()->getExpectedRawPredictionSetReader();
                    prediction_writer = EnvFactory::get()->getExpectedPredictionSetWriter();
                }
                motion_state_reader_ = EnvFactory::get()->getVehicleMotionStateReader();
                pLocalRoadMap_ = ParamsFactory::get()->getLocalRoadMap();
                pVehicle_ = ParamsFactory::get()->getVehicle();
                pPrediction_ = ParamsFactory::get()->getPrediction();

                blocking_states_.insert(adore::env::ConnectionState::STOP___AND___REMAIN);
                blocking_states_.insert(adore::env::ConnectionState::PRE___MOVEMENT);
            }
            void run()
            {
                roadmap_.update();
                three_lanes_.update();
                if(motion_state_reader_->hasData())
                {
                    adore::env::VehicleMotionState9d motion_state;
                    motion_state_reader_->getData(motion_state);
                    connectionSet_.update(pLocalRoadMap_->getDiscardRadius(),motion_state.getX(),motion_state.getY());

                    if(prediction_reader->hasUpdate())
                    {
                        adore::env::OccupancyCylinderPredictionSet predictionInputSet;//the input set
                        prediction_reader->getData(predictionInputSet);//read the input set
                        adore::mad::OccupancyCylinderTree prediction_blocker;//positions, which block predictions are accumulated here

                        adore::env::PrecedenceFilter precedenceFilter;/**< filters precedence rules on lanes*/
                        adore::env::PrecedenceFilter::TFilterResult relevant_rules;
                        //get all priority rules, which assign higher priority to left, right or center lane
                        //@TODO: decide whether to use separate prediction topics filtered for left/right/current lane planning
                        precedenceFilter.getHighPrioritiesOnLane(roadmap_.getPrecedenceSet(),three_lanes_.getCurrentLane(),relevant_rules);
                        precedenceFilter.getHighPrioritiesOnLane(roadmap_.getPrecedenceSet(),three_lanes_.getLeftLaneChange()->getTargetLane(),relevant_rules);
                        precedenceFilter.getHighPrioritiesOnLane(roadmap_.getPrecedenceSet(),three_lanes_.getRightLaneChange()->getTargetLane(),relevant_rules);
                        if(!worstcase_||pPrediction_->get_worstcase_filter_precedence())
                        {
                            for(auto rule:relevant_rules)
                            {
                                //place a blocker at low-priority connection start
                                adore::mad::OccupancyCylinder occupancy(0.5,rule->low_.from_(0),rule->low_.from_(1),-1.0e99,1.0e99,rule->low_.from_(2),rule->low_.from_(2)+3.0);
                                prediction_blocker.insert(occupancy);
                            }
                        }
                        //get controlled connections and block predictions which pass through connection-start-points associated with red-lights
                        if(!worstcase_||pPrediction_->get_worstcase_filter_tcd())
                        {
                            for(auto itpair = connectionSet_.getAllConnections();
                                itpair.current()!=itpair.end();
                                itpair.current()++)
                            {
                                adore::env::ControlledConnection* con = itpair.current()->second;
                                auto from = con->getID().getFrom();
                                //walk through all known phases
                                double t0 = motion_state.getTime()-5.0;
                                for(adore::env::ConnectionStateEvent& state:con->data_)
                                {
                                    double t1 = state.getMinEndTime();
                                    double nextt0 = state.getMaxOrMinEndTime();
                                    //if not green, create obstacle
                                    if(t1>t0 && blocking_states_.find(state.getState()) != blocking_states_.end())
                                    {
                                        adore::mad::OccupancyCylinder occupancy(0.5,from.get<0>(),from.get<1>(),t0,t1,from.get<2>()-2.0,from.get<2>()+3.0);
                                        prediction_blocker.insert(occupancy);
                                    }
                                    t0 = nextt0;
                                }
                            }
                        }
                        //protect ego against tail-gating predictions
                        if(three_lanes_.getCurrentLane()->isValid())
                        {
                            //compute point behind ego to serve as stopping point for potential "tail-gating" predictions
                            double s,n;
                            three_lanes_.getCurrentLane()->toRelativeCoordinates(motion_state.getX(),motion_state.getY(),s,n);
                            // //based on current lane, behind ego
                            // for(double ds=0.0;ds<5.0;ds+=1.0)
                            // {
                            //     double X,Y,Z;
                            //     three_lanes_.getCurrentLane()->toEucledianCoordinates(s-ds,n,X,Y,Z);                        
                            //     adore::mad::OccupancyCylinder occupancy(pVehicle_->get_bodyWidth(),X,Y,-1.0e99,1.0e99,Z-2.0,Z+3.0);
                            //     prediction_blocker.insert(occupancy);
                            // }
                            double Z = motion_state.getZ();
                            //rear end of ego
                            double X0 = motion_state.getX()-std::cos(motion_state.getPSI())*pVehicle_->get_d();
                            double Y0 = motion_state.getY()-std::sin(motion_state.getPSI())*pVehicle_->get_d();
                            //front end of ego
                            double X1 = motion_state.getX()+std::cos(motion_state.getPSI())*(pVehicle_->get_a()+pVehicle_->get_b()+pVehicle_->get_c());
                            double Y1 = motion_state.getY()+std::sin(motion_state.getPSI())*(pVehicle_->get_a()+pVehicle_->get_b()+pVehicle_->get_c());
                            double w = pVehicle_->get_bodyWidth()*0.5;
                            double r = w + 0.05;
                            double dr = std::sqrt(r*r-w*w);
                            double dx = X1-X0;
                            double dy = Y1-Y0;
                            double L = std::sqrt(dx*dx+dy*dy);
                            for(s = 0;s<=L*0.5;s+=2.0*dr)
                            {
                                adore::mad::OccupancyCylinder occupancy(r,X0+s/L*dx,Y0+s/L*dy,-1.0e99,1.0e99,-2.0,+3.0);//HeD, 20210521: TODO insert hight as soon as prediction provider also provides correct object hight
                                // adore::mad::OccupancyCylinder occupancy(r,X0+s/L*dx,Y0+s/L*dy,-1.0e99,1.0e99,Z-2.0,Z+3.0);
                                prediction_blocker.insert(occupancy);                            
                            }
                            
                        }

                        //find all prediction branches of the input set, which are colliding with a blocker: clip at collision time
                        std::unordered_map<int,double> clipped_branches;
                        std::unordered_set<int> blocked_branches;
                        for(auto& prediction:predictionInputSet)
                        {
                            double clip_time;
                            if(prediction.occupancy_.getEarliestCollisionTime(prediction_blocker,1.0e99,clip_time))
                            {
                                clipped_branches.emplace(std::make_pair(prediction.branchID_,clip_time));
                            }
                        }

                        //compute all successors of blocked or clipped branches
                        auto changed = true;
                        while(changed)
                        {
                            changed = false;
                            for(const auto& prediction:predictionInputSet)
                            {
                                if(    ( clipped_branches.find(prediction.predecessorID_)!=clipped_branches.end()
                                    ||   blocked_branches.find(prediction.predecessorID_)!=blocked_branches.end() )
                                    &&  blocked_branches.find(prediction.branchID_)==blocked_branches.end())
                                {
                                    changed = true;
                                    blocked_branches.emplace(prediction.branchID_);
                                }
                            }
                        }

                        //gather all predictions, which are neither blocked nor clipped
                        adore::env::OccupancyCylinderPredictionSet predictionOutputSet;//the filtered output set
                        for(const auto& prediction:predictionInputSet)
                        {
                            if(    blocked_branches.find(prediction.branchID_)==blocked_branches.end()
                                && clipped_branches.find(prediction.branchID_)==clipped_branches.end() )
                            {
                                predictionOutputSet.push_back(prediction);
                            }
                        }
                        //gather clipped predictions
                        for(const auto& prediction:predictionInputSet)
                        {
                            auto branch_clipped = clipped_branches.find(prediction.branchID_);
                            if( branch_clipped!=clipped_branches.end() 
                                && blocked_branches.find(prediction.branchID_)==blocked_branches.end() )
                            {
                                double clip_time = branch_clipped->second;
                                adore::env::OccupancyCylinderPrediction clipped_prediction;
                                clipped_prediction.branchID_ = prediction.branchID_;
                                clipped_prediction.trackingID_ = prediction.trackingID_;
                                clipped_prediction.predecessorID_ = prediction.predecessorID_;
                                clipped_prediction.confidence_ = prediction.confidence_;
                                for(const auto& indexed_cylinder:prediction.occupancy_.getLevel(0))
                                {
                                    const auto& cylinder = indexed_cylinder.second;
                                    if(cylinder.t0_<clip_time)
                                    {
                                        clipped_prediction.occupancy_.insert(cylinder);
                                    }
                                }
                                if(clipped_prediction.occupancy_.getOccupancyCount()>0)
                                {
                                    predictionOutputSet.push_back(clipped_prediction);
                                }
                            }
                        }

                        //send gathered predictions
                        prediction_writer->write(predictionOutputSet);
                    }


                }
            }            
        };

    }
}