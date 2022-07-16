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

#include <adore/fun/tac/planning_result.h>
#include <vector>
#include <list>

namespace adore
{
  namespace fun
  {
    /**
     * Interface to trajectory evaluation
     */

    using PlanningResultMap = std::unordered_map<int,adore::fun::PlanningResult>;

    class ATrajectoryEvaluator
    {
      protected:


      public:
        /**
         * evaluate - grade a trajectory and return a single value to compare with other trajectories
         */
        // TODO maybe it's better to leave these pure virtual and set them = 0
        virtual std::vector<double> evaluateToVector(const PlanningResultMap & planning_results, const std::vector<double> & weights) {return std::vector<double>();};
        virtual std::vector<double> evaluateToVector(const PlanningResultMap & planning_results) { return std::vector<double>();};
        virtual int evaluateToBest(const PlanningResultMap & planning_results, const std::vector<double> & weights) {return -1;};
        virtual int evaluateToBest(const PlanningResultMap & planning_results) {return -1;};
        
        ATrajectoryEvaluator ()
        {
          
        }
    };

  }
}