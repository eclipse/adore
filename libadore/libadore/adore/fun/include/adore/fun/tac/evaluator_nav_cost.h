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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore/fun/tac/atrajectory_evaluator.h>

namespace adore
{
    namespace fun
    {
        /**
         * Interface to trajectory evaluation
         */
        class EvaluatorNavCost : public ATrajectoryEvaluator
        {
          private:
            // TODO there should be a max nav cost at a convenient location in the source code
            double const MAX_NAV_COST = 40*1000*1000.0; // 40 000 km should suffice for even the wildest real world maps, TODO: unify with Borderbased code

          public:
            /**
             * evaluate - assign navigation cost of last point in SetPointRequest
             */
            int evaluateToBest(const PlanningResultMap & planning_results) override
            {
                const std::string target_field = "MinimumNavigationCostOnLane";
                int best_id = -1; // TODO negative value could be implicit error code
                double best_cost = MAX_NAV_COST;
                for (auto [id, plan] : planning_results)
                {
                    try
                    {
                        double plan_cost = plan.objective_values[target_field];
                        if (plan_cost < best_cost)
                        {
                            best_id = id;
                            best_cost = plan_cost;
                        }
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << "PlanningResult without value for "<<target_field<<" \n";
                    }

                }

                return best_id;
            }

            EvaluatorNavCost()
            {
            }
        };

    }  // namespace fun
}  // namespace adore