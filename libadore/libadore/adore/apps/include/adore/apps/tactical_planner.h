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
 *              - integrated dispatcher
 ********************************************************************************/

#pragma once

#include <adore/env/afactory.h>
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
#include <adore/fun/setpointrequest_dispatcher.h>
#include <adore/fun/indicator_dispatcher.h>
#include <adore/fun/turn_signal_observer.h>
#include <adore/fun/user_input_observer.h>
#include <adore/fun/turnstate.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief Decision making and maneuver planning
         */
        class TacticalPlanner
        {
            adore::params::APNavigation* pNavigation_;                      /**<parameters*/
            adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;  /**<parameters*/
            adore::params::APTacticalPlanner* pTacticalPlanner_;            /**<parameters*/
            adore::fun::AFactory::TMotionStateReader* vehicle_state_reader_;/**<get the time stamp of latest vehicle state*/
            adore::fun::AFactory::TPlanningRequestWriter* request_writer_;   /**<formulate planning requests for planning processes*/
            adore::fun::AFactory::TPlanningResultFeed* result_reader_;       /**<retrieve result of planning processes*/
            adore::fun::AFactory::TPlanningResultWriter* select_writer_;       /**<publish selected result of planning processes*/
            adore::env::AFactory::TPropositionWriter* proposition_writer_; /**<output proposition VALID_MANEUVER_AVAILABLE*/
            adore::fun::SetPointRequestDispatcher spr_dispatcher_;          /**<publish setpointrequests and retrieve initial state for next planning iteration*/
            adore::fun::IndicatorDispatcher indicator_dispatcher_;          /**<publish indicator command based on vehicle state and selected maneuver*/
            adore::env::AFactory::TNavigationGoalReader* navigation_goal_reader_; /**<track navigation goal updates: on change reset cost bound*/
            adore::env::AFactory::TResetLaneMatchingWriter* lane_view_reset_writer_; /**<send signal for lane matching reset*/
            uint64_t iteration_;
            adore::fun::EvaluatorWeightedSum evaluator_;        /**<evaluate trajectories with weighted sum of individual cost functions*/
            adore::fun::PlanningResult best_planning_result_;   /**<last active trajectory: might be active for >1 iterations, when no new, valid trajectory is found*/
            double iteration_time_length_;                      /**<duration of one iteration*/
            double last_time_;                                  /**<time of last planning iteration*/
            double cost_bound_;                                 /**<the current bound on navigation cost*/
            double cost_bound_guard_;                           /**<maximum value of navigation cost*/
            // TODO compound the following into a user input observer object
            double lanechange_supression_timeout_;              /**timeout for the lanechange suppression*/
            double force_langechange_left_timeout_;             /**timeout for force langechange left*/
            double force_langechange_right_timeout_;            /**timeout for force langechange right*/
            double force_slow_maneuver_timeout_;                /**timeout for the lanechange suppression*/
            std::string cost_bound_name_;                       /**<navigation cost name*/
            fun::TurnSignalObserver turn_signal_observer_;           /**<tracking of turn signal changes*/ 
            fun::UserInputObserver user_input_observer_;           /**<handling user input*/ 
            adore::fun::AFactory::TLanechangeSuppressionReader* langechange_suppression_reader_; // get signals about lanechange suppression
            adore::fun::AFactory::TForceLanechangeLeftReader* force_langechange_left_reader_; // get signals about lanechange suppression
            adore::fun::AFactory::TForceLanechangeRightReader* force_langechange_right_reader_; // get signals about lanechange suppression
            adore::fun::AFactory::TForceSlowManeuversReader* force_slow_maneuvers_reader_; // get signals about lanechange suppression

          public:
            adore::fun::EvaluatorWeightedSum* getEvaluator(){return &evaluator_;}

            /**
             * @brief constructur
             * @param iteration_time_length provide information how long iteration should take: influences planning request
             */
            TacticalPlanner(double iteration_time_length)
            {
                iteration_time_length_ = iteration_time_length;
                iteration_ = 0u;
                pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
                pNavigation_ = adore::params::ParamsFactoryInstance::get()->getNavigation();
                pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
                request_writer_ = adore::fun::FunFactoryInstance::get()->getPlanningRequestWriter();
                result_reader_ = adore::fun::FunFactoryInstance::get()->getPlanningResultFeed();
                select_writer_ = adore::fun::FunFactoryInstance::get()->getPlanningSelectWriter();
                vehicle_state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
                proposition_writer_ = adore::env::EnvFactoryInstance::get()->getPropositionWriter();
                navigation_goal_reader_ = adore::env::EnvFactoryInstance::get()->getNavigationGoalReader();
                lane_view_reset_writer_ = adore::env::EnvFactoryInstance::get()->getResetLaneMatchingWriter();
                langechange_suppression_reader_ = adore::fun::FunFactoryInstance::get()->getLanechangeSuppressionReader();
                force_langechange_left_reader_ = adore::fun::FunFactoryInstance::get()->getForceLanechangeLeftReader();
                force_langechange_right_reader_ = adore::fun::FunFactoryInstance::get()->getForceLanechangeRightReader();
                force_slow_maneuvers_reader_ = adore::fun::FunFactoryInstance::get()->getForceSlowManeuversReader();

                last_time_ = 0.0;
                lanechange_supression_timeout_ = 0.0;
                cost_bound_guard_ = 1.e99;
                cost_bound_ = cost_bound_guard_;
                cost_bound_name_ = "MinimumNavigationCostOnLane";//@TODO HeD 20210917: replace with parameter
            }

            virtual ~TacticalPlanner()
            {
                delete pTrajectoryGeneration_;
                delete request_writer_;
                delete result_reader_;
                delete vehicle_state_reader_;
                delete proposition_writer_;
            }

            /**
             * @brief retrieve planning results, dispatch and formulate new planning request
             *
             */
            void run()
            {
                //update current vehicle state
                if(!vehicle_state_reader_->hasData())return;
                adore::fun::VehicleMotionState9d x;
                vehicle_state_reader_->getData(x);
                if(x.getTime()==last_time_)
                {
                    return;
                }
                //else
                last_time_ = x.getTime();

                if (x.getvx()<pTacticalPlanner_->getLVResetVelocity())
                {
                    lane_view_reset_writer_->write(true);
                }
                

                //retrieve planning results received before current iteration
                adore::fun::PlanningResultMap valid_results;
                std::vector<adore::fun::PlanningResult> available_results;
                adore::fun::Turnstate turnstate;
                turnstate = adore::fun::Turnstate::off;
                if (turn_signal_observer_.rightIndicatorTurnedOnManuallyWithinLastSecond(last_time_,pTacticalPlanner_->getTimeoutForPreferredLCAfterManuallySetIndicator()))
                {
                    turnstate = adore::fun::Turnstate::right;
                }
                else if (turn_signal_observer_.leftIndicatorTurnedOnManuallyWithinLastSecond(last_time_,pTacticalPlanner_->getTimeoutForPreferredLCAfterManuallySetIndicator()))
                {
                    turnstate = adore::fun::Turnstate::left;
                }

                // handle signal to suppress lanechanges
                bool suppress_lanechanges = false;
                if (langechange_suppression_reader_->hasUpdate())
                {
                    // if (x.getTime() > lanechange_supression_timeout_)
                    // {
                        langechange_suppression_reader_->getData(suppress_lanechanges);
                        if (suppress_lanechanges) // should always be true, but just to be sure check this
                        {
                            lanechange_supression_timeout_ = x.getTime() + pTacticalPlanner_->getTimeoutForLangechangeSuppression();
                            force_langechange_left_timeout_ = x.getTime();
                            force_langechange_right_timeout_ = x.getTime();
                        }
                    // }
                    // else
                    // {
                    //     std::cout << "Ignored Lanechange Suppression Signal due to timeout"
                    // }
                }
                if (x.getTime() < lanechange_supression_timeout_)
                {
                    // if the time is before the timeout, suppression is still true
                    suppress_lanechanges = true;
                }

                bool force_lanechange_left = false;
                if (force_langechange_left_reader_->hasUpdate())
                {
                    force_langechange_left_reader_->getData(force_lanechange_left);
                    if (force_lanechange_left) // should always be true, but just to be sure check this
                    {
                        force_langechange_left_timeout_ = x.getTime() + pTacticalPlanner_->getTimeoutForPreferredLCAfterManuallySetIndicator();
                    }
                }
                if (x.getTime() < force_langechange_left_timeout_)
                {
                    force_lanechange_left = true;
                }

                bool force_lanechange_right = false;
                if (force_langechange_right_reader_->hasUpdate())
                {
                    force_langechange_right_reader_->getData(force_lanechange_right);
                    if (force_lanechange_right) // should always be true, but just to be sure check this
                    {
                        force_langechange_right_timeout_ = x.getTime() + pTacticalPlanner_->getTimeoutForPreferredLCAfterManuallySetIndicator();
                        lane_view_reset_writer_->write(true);
                    }
                }
                if (x.getTime() < force_langechange_right_timeout_)
                {
                    force_lanechange_right = true;
                }

                bool force_slow_maneuvers = false;
                if (force_slow_maneuvers_reader_->hasUpdate())
                {
                    force_slow_maneuvers_reader_->getData(force_slow_maneuvers);
                    if (force_slow_maneuvers) // should always be true, but just to be sure check this
                    {
                        force_slow_maneuver_timeout_ = x.getTime() + pTacticalPlanner_->getTimeoutForPreferredLCAfterManuallySetIndicator();
                        lane_view_reset_writer_->write(true);
                    }
                }
                if (x.getTime() < force_slow_maneuver_timeout_)
                {
                    force_slow_maneuvers = true;
                }

                // if (turn_signal_observer_.newManualLeftIndicatorOnEvent() || turn_signal_observer_.newManualRightIndicatorOnEvent())
                // {
                //     lane_view_reset_writer_->write(true);
                // }

                while(result_reader_->hasNext())
                {
                    adore::fun::PlanningResult result;
                    result_reader_->getNext(result);
                    bool valid = true;
                    bool isLCL = false;
                    bool isLCR = false;



                    if(result.iteration != iteration_)
                    { 
                        result.status_string = "wrong iteration; "+result.status_string;
                        valid = false;
                    }
                    if(!result.combined_maneuver_valid)
                    {
                        valid = false;
                    }
                    if(!(result.getObjectiveValue(cost_bound_name_,cost_bound_guard_)<=cost_bound_))//expr. !(.<=.) is used to filter nan values
                    {
                        result.status_string = "cost bound exceeded; "+result.status_string;
                        valid = false;
                    }

                    if(result.name.find("lcr") != std::string::npos)
                    {
                        isLCR = true;
                    }
                    else if(result.name.find("lcl") != std::string::npos)
                    {
                        isLCL = true;
                    }

                    if(suppress_lanechanges && (isLCL || isLCR))
                    {
                        valid = false;
                        result.status_string = "suppressed lc maneuver; "+result.status_string;
                    }
                    else
                    {
                        // if(turnstate ==  adore::fun::Turnstate::left && result.name.find("lcl") == std::string::npos)
                        if(force_lanechange_left && result.name.find("lcl") == std::string::npos)
                        {
                            valid = false;
                            result.status_string = "non-lcl maneuver; "+result.status_string;
                        }
                        // if(turnstate ==  adore::fun::Turnstate::right && result.name.find("lcr") == std::string::npos)
                        if(force_lanechange_right && result.name.find("lcr") == std::string::npos)
                        {
                            valid = false;
                            result.status_string = "non-lcr maneuver; "+result.status_string;
                        }
                    }

                    available_results.push_back(result);
                    if(valid)valid_results.insert({result.id,result});
                }


                //decide which planning result to execute
                static_cast<adore::fun::EvaluatorWeightedSum>(evaluator_).setTurnState(turnstate);
                int best_id = evaluator_.evaluateToBest(valid_results);
                if(valid_results.size()>0 && best_id>=0)
                {
                    best_planning_result_ = valid_results[best_id];
                    select_writer_->write(best_planning_result_);
                    spr_dispatcher_.dispatch(best_planning_result_.combined_maneuver,
                                         best_planning_result_.nominal_maneuver);
                    adore::env::Proposition prop("VALID_MANEUVER_AVAILABLE",true);
                    proposition_writer_->write(prop);

                    if( pTacticalPlanner_->getEnforceMonotonousNavigationCost() )
                    {
                        cost_bound_ = best_planning_result_.getObjectiveValue(cost_bound_name_,cost_bound_guard_)
                                    + pNavigation_->getLaneChangePenalty() * 0.5; //add 1/2 lane change penalty for robustness in cost fluctuation
                    }
                }
                else
                {
                    adore::env::Proposition prop("VALID_MANEUVER_AVAILABLE",false);
                    proposition_writer_->write(prop);
                }
                indicator_dispatcher_.setIndicators(best_planning_result_);
            

                //status output: planning result overview
                std::cout<<std::endl<<"--------------------------------------------------------------------------------"<<std::endl;
                if(valid_results.size()==0)std::cout << "No valid plans\n";
                std::cout<<"Current cost bound ("<<cost_bound_name_<<"): "<<cost_bound_<<std::endl;
                std::sort(std::begin(available_results), 
                        std::end(available_results), 
                        [](auto& a, auto& b) {return a.id < b.id; });                
                for(auto& result:available_results)
                {
                    std::cout<<((result.id==best_planning_result_.id && result.iteration==best_planning_result_.iteration)?"*":" ");
                    std::cout<<result.name<<" ("<<result.nominal_maneuver_valid<<","<<result.combined_maneuver_valid<<") ";
                    std::cout<<"c="<<evaluator_.getCost(result.id)<<" ";
                    std::cout<<"c_ttc_nom="<<result.getObjectiveValue("TTC_nom",0.0)<<" ";
                    std::cout<<result.status_string<<std::endl;
                }

                //compute initial state and formulate request, to be evaluated in next iteration
                adore::fun::VehicleMotionState9d x0;
                bool x0_available = spr_dispatcher_.getInitialState(x0);
                if(spr_dispatcher_.getStatus()!="")std::cout<<spr_dispatcher_.getStatus()<<std::endl;
                if(!x0_available) return;//not able to formulate request in this iteration
                adore::fun::PlanningRequest req;
                req.iteration = ++iteration_;
                req.initial_state.x0ref = adore::fun::PlanarVehicleState10d( x0 );
                if(req.initial_state.x0ref.getvx()<0.1 && req.initial_state.x0ref.getAx()<0.0)req.initial_state.x0ref.setAx(0.0);
                req.initial_state.x0ref.setDAx(0.0);
                req.initial_state.x0ref.setDDelta(0.0);
                req.initial_state.tStart = x0.getTime();
                req.initial_state.tEnd = req.initial_state.tStart;
                req.t_emergency_start = req.initial_state.tStart + pTrajectoryGeneration_->getEmergencyManeuverDelay();
                req.t_planning_start = req.initial_state.tStart; // TODO is "now" a good time?
                req.t_planning_end = req.t_planning_start + iteration_time_length_ * 0.95; // TODO should there be a better parametrization? magic numbers warning
                request_writer_->write(req);

                //update navigation cost -> if received, reset cost_bound_
                if(navigation_goal_reader_->hasUpdate())
                {
                    adore::fun::NavigationGoal goal;
                    navigation_goal_reader_->getData( goal );//the goal itself is not important, just reset
                    cost_bound_ = cost_bound_guard_;
                }
                
                turn_signal_observer_.update(last_time_);

            }
        };
    }  // namespace apps
}  // namespace adore
