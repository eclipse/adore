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
 *   Thomas Lobig - initial implementation
 *   Daniel Heß - added automatic control/reset logic, indicators, propositions
 *              - selection via parametrizable weighted sum
 ********************************************************************************/

#pragma once

#include <adore/apps/if_plotlab/plot_shape.h>
#include <adore/env/afactory.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/fun/afactory.h>
#include <adore/fun/tac/evaluator_nav_cost.h>
#include <adore/params/afactory.h>
#include <adore/view/alane.h>

#include <adore/fun/tac/planning_request.h>
#include <adore/fun/tac/planning_result.h>
#include <adore/fun/tac/atrajectory_evaluator.h>
#include <adore/fun/tac/evaluator_weighted_sum.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief Decision making and maneuver planning
         */
        class TacticalPlanner
        {
            using FunFactory = adore::fun::FunFactoryInstance;
            using ParamsFactory = adore::params::ParamsFactoryInstance;
            using Trajectory = adore::fun::SetPointRequest;
            using TrajectoryWriter = std::shared_ptr<adore::mad::AWriter<Trajectory>>;
            using VehicleStateReader = std::shared_ptr<adore::mad::AReader<adore::fun::VehicleMotionState9d>>;
            using VehicleExtendedStateReader = std::shared_ptr<adore::mad::AReader<adore::fun::VehicleExtendedState>>;
            using PlanningRequestWriter = std::shared_ptr<adore::mad::AWriter<adore::fun::PlanningRequest>>;
            using PlanningResultFeed = std::shared_ptr<adore::mad::AFeed<adore::fun::PlanningResult>>;
            using IndicatorCommandWriter = std::shared_ptr<adore::mad::AWriter<adore::fun::IndicatorCommand>>;
            using PropositionWriter = std::shared_ptr<adore::mad::AWriter<adore::env::Proposition>>;

            adore::params::APVehicle* pvehicle_;
            adore::params::APTacticalPlanner* pTacticalPlanner_;
            adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;            
            VehicleStateReader vehicle_state_reader_;
            VehicleExtendedStateReader vehicle_extended_state_reader_;
            PlanningRequestWriter request_writer;
            PlanningResultFeed result_reader;
            IndicatorCommandWriter indicator_writer_;
            PropositionWriter proposition_writer_;
            adore::mad::AWriter<Trajectory>* sprwriter_;
            adore::mad::AWriter<Trajectory>* ntwriter_;
            adore::fun::VehicleMotionState9d vehicle_state_;
            adore::fun::VehicleExtendedState vehicle_extended_state_;


            uint64_t iteration_;
            adore::fun::EvaluatorWeightedSum evaluator_;
            adore::fun::PlanningResult best_planning_result_;
            adore::fun::PlanningResultMap valid_results_;
            std::vector<adore::fun::PlanningResult> available_results_;
            double iteration_time_length_;/**<duration of one iteration*/
            adore::fun::SetPoint last_planning_start_point_;
            double last_time_;

          public:
            adore::fun::EvaluatorWeightedSum* getEvaluator(){return &evaluator_;}

            virtual ~TacticalPlanner()
            {
                // delete sprwriter_;
                // delete lf_planner_;
                // delete lcl_planner_;
                // delete lcr_planner_;
            }

            TacticalPlanner(double iteration_time_length)
            {
                iteration_time_length_ = iteration_time_length;
                iteration_ = 0u;
                pvehicle_ = ParamsFactory::get()->getVehicle();
                pTacticalPlanner_ = ParamsFactory::get()->getTacticalPlanner();
                pTrajectoryGeneration_ = ParamsFactory::get()->getTrajectoryGeneration();
                request_writer = PlanningRequestWriter(adore::fun::FunFactoryInstance::get()->getPlanningRequestWriter());
                result_reader = PlanningResultFeed(adore::fun::FunFactoryInstance::get()->getPlanningResultFeed());
                vehicle_state_reader_ = std::shared_ptr<adore::mad::AReader<adore::fun::VehicleMotionState9d> >(FunFactory::get()->getVehicleMotionStateReader());
                vehicle_extended_state_reader_ = VehicleExtendedStateReader(FunFactory::get()->getVehicleExtendedStateReader());
                indicator_writer_ = IndicatorCommandWriter(adore::fun::FunFactoryInstance::get()->getIndicatorCommandWriter());
                proposition_writer_ = PropositionWriter(adore::env::EnvFactoryInstance::get()->getPropositionWriter());

                sprwriter_ = FunFactory::get()->getSetPointRequestWriter();
                ntwriter_ = FunFactory::get()->getNominalTrajectoryWriter();

                last_time_ = 0.0;
            }


            /**
             * @brief update data, views and recompute maneuver
             *
             */
            void run()
            {
                //update current vehicle state
                vehicle_state_reader_->getData(vehicle_state_);
                if(vehicle_state_.getTime()==last_time_)return;
                else last_time_ = vehicle_state_.getTime();

                vehicle_extended_state_reader_->getData(vehicle_extended_state_);
                bool automatic_control_enabled =   vehicle_extended_state_.getAutomaticControlOn() //< Freigabe im Fahrzeuginterface für Längs und Quer erhalten
                                                && vehicle_extended_state_.getAutomaticControlAccelerationActive();// Bestätigung der Freigabe durch Benutzer/Gaspedal erfolgt

                // ### evaluate planning results received during current iteration
                valid_results_.clear();
                while(result_reader->hasNext())
                {
                    adore::fun::PlanningResult result;
                    result_reader->getNext(result);

                    if(result.iteration != iteration_)result.status_string = "wrong iteration; "+result.status_string;
                    available_results_.push_back(result);

                    if(result.iteration == iteration_ && result.combined_maneuver_valid)
                    {
                        valid_results_.insert({result.id,result});
                    }
                }

                int best_id = evaluator_.evaluateToBest(valid_results_);
                if(valid_results_.size()>0 && best_id>=0)
                {
                    best_planning_result_ = valid_results_[best_id];
                    sprwriter_->write(best_planning_result_.combined_maneuver);
                    ntwriter_->write(best_planning_result_.nominal_maneuver);
                    adore::env::Proposition prop("VALID_MANEUVER_AVAILABLE",true);
                    proposition_writer_->write(prop);
                }
                else
                {
                    adore::env::Proposition prop("VALID_MANEUVER_AVAILABLE",false);
                    proposition_writer_->write(prop);
                }
            

                //debug/status output
                std::cout<<std::endl<<"--------------------------------------------------------------------------------"<<std::endl;
                if(valid_results_.size()==0)std::cout << "No valid plans\n";
                std::sort(std::begin(available_results_), 
                        std::end(available_results_), 
                        [](auto& a, auto& b) {return a.id < b.id; });                
                for(auto& result:available_results_)
                {
                    std::cout<<((result.id==best_planning_result_.id && result.iteration==best_planning_result_.iteration)?"*":" ");
                    std::cout<<result.name<<" ("<<result.nominal_maneuver_valid<<","<<result.combined_maneuver_valid<<") ";
                    // std::cout<<"nnc="<<result.objective_values["NormalizedNavigationCost"]<<" ";
                    std::cout<<"mnc="<<result.objective_values["MinimumNavigationCostOnLane"]<<" ";
                    std::cout<<result.status_string<<std::endl;
                }

                //set indicators only if automatic control enabled
                if( automatic_control_enabled )
                {
                    if( best_planning_result_.combined_maneuver_valid 
                    &&  best_planning_result_.combined_maneuver.isActive(vehicle_state_.getTime()) )
                    {
                        adore::fun::IndicatorCommand ic;
                        ic.setIndicatorLeftOn(best_planning_result_.indicator_left);
                        ic.setIndicatorRightOn(best_planning_result_.indicator_right);
                        indicator_writer_->write(ic);
                    }
                    else
                    {
                        adore::fun::IndicatorCommand ic;
                        if(vehicle_state_.getvx()>2.0)
                        {
                            //if no maneuver is active, set emergency indicators
                            ic.setIndicatorLeftOn(true);
                            ic.setIndicatorRightOn(true);                    
                        }
                        else
                        {
                            ic.setIndicatorLeftOn(false);
                            ic.setIndicatorRightOn(false);
                        }
                        indicator_writer_->write(ic);
                    }
                }
                else
                {
                    //if in manual mode, always set automatic control of indicators to false
                    adore::fun::IndicatorCommand ic;
                    ic.setIndicatorLeftOn(false);
                    ic.setIndicatorRightOn(false);
                    indicator_writer_->write(ic);
                }

                // ### setup next planning iteration                
                adore::fun::SetPoint planning_start_point;
                // planning_start_point.tStart = vehicle_state_.getTime() + iteration_time_length_ ;
                planning_start_point.tStart = vehicle_state_.getTime() ;
                planning_start_point.tEnd = planning_start_point.tStart;

                bool reset = true; 
                if( best_planning_result_.combined_maneuver_valid 
                &&  best_planning_result_.combined_maneuver.isActive(planning_start_point.tStart) )
                {
                    auto x_ref = best_planning_result_.combined_maneuver.interpolateReference(planning_start_point.tStart, pvehicle_);
                    double dx = vehicle_state_.getX() - x_ref.getX();
                    double dy = vehicle_state_.getY() - x_ref.getY();
                    double R = pTacticalPlanner_->getResetRadius();
                    if (dx * dx + dy * dy < R*R)
                    {
                        planning_start_point.x0ref = x_ref;
                        reset = false;
                    }
                    else
                    {
                        std::cout<<"Reset radius exceeded"<<std::endl;
                    }
                }
                else
                {
                    if(!best_planning_result_.combined_maneuver_valid)
                    {
                        std::cout<<"The selected maneuver is not valid"<<std::endl;
                    }
                    if(!best_planning_result_.combined_maneuver.isActive(planning_start_point.tStart))
                    {
                        if(best_planning_result_.combined_maneuver.setPoints.size()==0)
                        {
                            std::cout<<"The selected maneuver has no set points"<<std::endl;
                        }
                        else
                        {
                            std::cout<<"The selected maneuver ["
                                     << best_planning_result_.combined_maneuver.setPoints[0].tStart
                                     << "," 
                                     << best_planning_result_.combined_maneuver.setPoints[best_planning_result_.combined_maneuver.setPoints.size()-1].tEnd 
                                     << "is not active for time "<<planning_start_point.tStart<<std::endl;
                            
                        }
                        

                    }
                }

                //while in manual control mode, always reset initial state to current vehicle state
                if(!automatic_control_enabled)
                {
                    reset = true;
                }
                
                
                if (reset)
                {
                    planning_start_point.x0ref = vehicle_state_;
                    planning_start_point.tStart = vehicle_state_.getTime();
                    planning_start_point.tEnd = planning_start_point.tStart;
                    std::cout << "Resetting initial state for next planning iteration.\n";
                }

                valid_results_.clear();
                available_results_.clear();

                adore::fun::PlanningRequest req;

                planning_start_point.x0ref.setDAx(0.0);
                planning_start_point.x0ref.setDDelta(0.0);
                req.initial_state = planning_start_point;
                iteration_++;
                req.iteration = iteration_;
                req.t_planning_start = vehicle_state_.getTime(); // TODO is "now" a good time?
                req.t_planning_end = req.t_planning_start + iteration_time_length_ * 0.95; // TODO should there be a better parametrization? magic numbers warning
                req.t_emergency_start = planning_start_point.tStart + pTrajectoryGeneration_->getEmergencyManeuverDelay(); 
                request_writer->write(req);

                bool log_transformation = false;
                if(log_transformation)
                {
                    std::cout<<"initial state difference:"<<std::endl;
                    std::cout<<"dX="<<planning_start_point.x0ref.getX()-last_planning_start_point_.x0ref.getX()<<std::endl;
                    std::cout<<"dY="<<planning_start_point.x0ref.getY()-last_planning_start_point_.x0ref.getY()<<std::endl;
                    std::cout<<"dPSI="<<planning_start_point.x0ref.getPSI()-last_planning_start_point_.x0ref.getPSI()<<std::endl;
                    std::cout<<"dvx="<<planning_start_point.x0ref.getvx()-last_planning_start_point_.x0ref.getvx()<<std::endl;
                    std::cout<<"dvy="<<planning_start_point.x0ref.getvy()-last_planning_start_point_.x0ref.getvy()<<std::endl;
                    std::cout<<"domega="<<planning_start_point.x0ref.getOmega()-last_planning_start_point_.x0ref.getOmega()<<std::endl;
                    std::cout<<"dAx="<<planning_start_point.x0ref.getAx()-last_planning_start_point_.x0ref.getAx()<<std::endl;
                    std::cout<<"dDelta="<<planning_start_point.x0ref.getDelta()-last_planning_start_point_.x0ref.getDelta()<<std::endl;
                }
                last_planning_start_point_ = planning_start_point;
            }
        };
    }  // namespace apps
}  // namespace adore
