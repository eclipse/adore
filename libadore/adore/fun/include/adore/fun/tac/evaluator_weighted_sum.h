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
#include <vector>
#include <string>

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

          public:
            EvaluatorWeightedSum():max_cost_(1.0e99){}
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
                    for( auto o: objectives_)//go through relevant objectives
                    {
                        if(o.second>0.0)//if the weight is greater then zero
                        {
                            try
                            {
                                weighted_sum += o.second * plan.objective_values[o.first];
                            }
                            catch(...)
                            {
                                all_objectives_present = false;
                            }
                        }
                    }
                    if( all_objectives_present && 0.0<=weighted_sum && weighted_sum<max_cost_ )
                    {
                        cost_list_.push_back(std::make_pair(id,weighted_sum));
                    }
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