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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/params/ap_lateral_planner.h>

namespace adore
{
  namespace params
  {
    /**
     * @brief a dummy implementation for testing purposes
     * 
     */
    class APLateralPlannerDummy: public APLateralPlanner
    {
      public:
      ///getWeightPos returns cost function weight for quadratic position error term
      virtual double getWeightPos() const 
      {
        return 6.0;
      }
      ///getWeightVel returns cost function weight for quadratic velocity error term
      virtual double getWeightVel() const 
      {
        return 1.0;
      }

      ///getWeightAcc returns cost function weight for quadratic acceleration term
      virtual double getWeightAcc() const 
      {
        return 1.0;
      }

      ///getWeightJerk returns cost function weight for quadratic jerk term
      virtual double getWeightJerk() const 
      {
        return 1.0;
      }

      ///getWeightEndPos returns cost function weight for quadratic position error term at end point
      virtual double getWeightEndPos() const 
      {
        return 0.0;
      }

      ///getWeightEndVel returns cost function weight for quadratic velocity error term at end point
      virtual double getWeightEndVel() const 
      {
        return 0.0;
      }

      ///getWeightEndAcc returns cost function weight for quadratic acceleration term at end point
      virtual double getWeightEndAcc() const 
      {
        return 0.0;
      }

      ///getSlackPos returns maximum slack of soft-constraints for position
      virtual double getSlackPos() const 
      {
        return 0.1;
      }

      ///getSlackVel returns maximum slack of soft-constraints for velocity
      virtual double getSlackVel() const 
      {
        return 0.1;
      }

      ///getSlackAcc returns maximum slack of soft-constraints for acceleration
      virtual double getSlackAcc() const 
      {
        return 0.1;
      }

      ///getAccLB returns lateral acceleration lower bound
      virtual double getAccLB() const
      {
        return -10.0;
      }

      ///getAccUB returns lateral acceleration upper bound
      virtual double getAccUB() const
      {
        return 10.0;
      }

      ///getJerkLB returns lateral jerk lower bound
      virtual double getJerkLB() const
      {
        return -100.0;
      }

      ///getJerkLB returns lateral jerk upper bound
      virtual double getJerkUB() const
      {
        return 100.0;
      }

      ///getCurvatureUB returns maximum curvature of path (relevant at low speeds)
      virtual double getCurvatureUB() const
      {
        return 1.0/7.0;
      }

      ///getCurvatureLB returns minimum curvature of path (relevant at low speeds)
      virtual double getCurvatureLB() const
      {
        return -1.0/7.0;
      }

      ///getRelativeHeadingUB returns upper bound on heading deviation from current lane's coordinate system
      virtual double getRelativeHeadingUB() const
      {
        return 0.7;
      }

      ///getRelativeHeadingLB returns lower bound on heading deviation from current lane's coordinate system
      virtual double getRelativeHeadingLB() const
      {
        return -0.7;
      }

      ///getMergeConstraintDelay returns a time-delay after which lateral position constraints are activated, if they are initially violated
      virtual double getMergeConstraintDelay() const
      {
        return 3.0;
      }

      ///getMaxCPUTime returns the maximum cpu time for one plan computation
      virtual double getMaxCPUTime() const
      {
        return 0.05;
      }

    };
  }
}