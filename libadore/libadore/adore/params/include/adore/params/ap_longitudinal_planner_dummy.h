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
#include <adore/params/ap_longitudinal_planner.h>

namespace adore
{
  namespace params
  {
    /**
     * @brief a dummy implementation for testing purposes
     * 
     */
    class APLongitudinalPlannerDummy: public APLongitudinalPlanner
    {

      public:
      ///getWeightPos returns cost function weight for quadratic position error term
      virtual double getWeightPos() const
      {
        return 2.0;
      }
      ///getWeightVel returns cost function weight for quadratic velocity error term
      virtual double getWeightVel() const 
      {
        return 5.0;
      }

      ///getWeightAcc returns cost function weight for quadratic acceleration term
      virtual double getWeightAcc() const 
      {
        return 5.0;
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
        return 0.5;
      }

      ///getSlackVel returns maximum slack of soft-constraints for velocity
      virtual double getSlackVel() const 
      {
        return 0.01;
      }

      ///getSlackAcc returns maximum slack of soft-constraints for acceleration
      virtual double getSlackAcc() const 
      {
        return 5.0;
      }

      ///getAccLB returns longitudinal acceleration lower bound
      virtual double getAccLB() const
      {
        return -3.0;
      }

      ///getAccUB returns longitudinal acceleration upper bound
      virtual double getAccUB() const
      {
        return 2.0;
      }


      ///getAccLB returns longitudinal acceleration lower bound
      virtual double getComfortAccLB() const override
      {
        return -1.0;
      }
      
      ///getAccUB returns longitudinal acceleration upper bound
      virtual double getComfortAccUB() const override
      {
        return 1.0;
      }

      ///getJerkLB returns longitudinal jerk lower bound
      virtual double getJerkLB() const
      {
        return -100.0;
      }

      ///getJerkLB returns longitudinal jerk upper bound
      virtual double getJerkUB() const
      {
        return 100.0;
      }

      ///getAccLatUB returns the absolute lateral acceleration bound which has to be maintained by reducing speed
      virtual double getAccLatUB() const
      {
        return 2.0;
      }

      ///getAccLatUB_minVelocity returns the minimum velocity, which is always feasible despite getAccLatUB
      virtual double getAccLatUB_minVelocity() const
      {
        return 1.0;
      }

      ///getConstraintClearancePos returns the offset of the reference from the position constraints
      virtual double getConstraintClearancePos() const
      {
        return 0.1;
      }

      ///getConstraintClearanceVel returns the offset of the reference from the velocity constraints
      virtual double getConstraintClearanceVel() const
      {
        return 0.05;
      }

      ///getMaxCPUTime returns the maximum cpu time for one plan computation
      virtual double getMaxCPUTime() const
      {
        return 0.05;
      }

			/**
			 * @brief distance between stop position and conflict point
			 */
			virtual double getStopDistanceToConflictPoint()const
      {
        return 5.0;
      }
    };
  }
}