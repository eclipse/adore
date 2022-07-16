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

#include <adore/fun/tac/atrajectory_evaluator.h>
#include <adore/fun/turnstate.h>
#include <vector>
#include <string>
#include <limits>

namespace adore
{
    namespace fun
    {
        /**
         * A weighted sum of individual objective functions is used to select the best trajectory.
         */
        class EvaluatorWeightedSum : public ATrajectoryEvaluator
        {
          private:
            std::vector<std::pair<std::string,double>> objectives_;
            std::vector<std::pair<int,double>> cost_list_;
            double max_cost_;
            Turnstate turnstate_;


          public:
            
            EvaluatorWeightedSum():max_cost_(1.0e99){}

            void setTurnState(Turnstate state)
            {
                turnstate_ = state;
            }
            void addParameterPair(std::string name,std::string weight_string)
            {
                double weight = 0.0;
                try{weight = std::stod(weight_string);}catch(...){}
                objectives_.push_back(std::make_pair(name,weight));
            }
            void init()
            {
                if(objectives_.size()==0)
                {
                    objectives_.push_back(std::make_pair("MinimumNavigationCostOnLane",1.0));
                }
            }
            /**
             * @return cost of plan with given id
             */
            double getCost(int id)
            {
                int idx = getIndex(id);
                if(idx==-1)return std::numeric_limits<double>::quiet_NaN();
                else return cost_list_[idx].second;
            }
            /**
             * @return index in cost_list_ for plan with id, -1 if not found
             */
            int getIndex(int id)
            {
                for(int i=0;i<cost_list_.size();i++)
                {
                    if(cost_list_[i].first==id)return i;
                }
                return -1;
            }
            /**
             * evaluate - assign navigation cost of last point in SetPointRequest
             */
            int evaluateToBest(const PlanningResultMap & planning_results) override
            {
                init();
                cost_list_.clear();
                for (auto [id, plan] : planning_results)//go through results
                {
                    bool all_objectives_present = true;
                    double weighted_sum = 0.0;
                    for( auto [objective_name,objective_weight]: objectives_)//go through relevant objectives
                    {
                        if(objective_weight>0.0)//if the weight is greater then zero
                        {
                            try
                            {
                                weighted_sum += objective_weight * plan.objective_values[objective_name];
                            }
                            catch(...)
                            {
                                all_objectives_present = false;
                                std::cout<<"plan "<<plan.name<<" missing objective "<<objective_name<<"."<<std::endl;
                            }
                        }
                    }
                    //temporal solution: flat bonus if indicator matches
                    if ( plan.indicator_left && turnstate_ == Turnstate::left)
                    {
                        weighted_sum += 2001.0;
                    }
                    if ( plan.indicator_right && turnstate_ == Turnstate::right)
                    {
                        weighted_sum += 2001.0;
                    }


                    if( !all_objectives_present )
                    {
                        std::cout<<"plan "<<plan.name<<" missing objectives."<<std::endl;
                        continue;
                    }
                    if(!(weighted_sum>=0.0))
                    {
                        std::cout<<"plan "<<plan.name<<" weighted_sum not positive: "<<weighted_sum<<"."<<std::endl;
                        continue;
                    }
                    if(!(weighted_sum<max_cost_))
                    {
                        std::cout<<"plan "<<plan.name<<" weighted_sum not smaller max_cost: "<<weighted_sum<<">="<<max_cost_<<"."<<std::endl;
                        continue;
                    }
                    cost_list_.push_back(std::make_pair(id,weighted_sum));
                }
                if(cost_list_.size()==0)return -1;
                double min_id = cost_list_[0].first;
                double min_value = cost_list_[0].second;
                for(int i=1;i<cost_list_.size();i++)
                {
                    if(cost_list_[i].second<min_value)
                    {
                        min_id = cost_list_[i].first;
                        min_value = cost_list_[i].second;
                    }
                }
                return min_id;
            }

        };

    }  // namespace fun
}  // namespace adore