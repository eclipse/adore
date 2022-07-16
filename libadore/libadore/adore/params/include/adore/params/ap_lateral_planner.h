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

namespace adore
{
  namespace params
  {
    /**
     * @brief abstract class containing parameters related to configuring the lateral planner
     * 
     */
    class APLateralPlanner
    {
      public:
      ///getWeightPos returns cost function weight for quadratic position error term
      virtual double getWeightPos() const =0;

      ///getWeightVel returns cost function weight for quadratic velocity error term
      virtual double getWeightVel() const =0;

      ///getWeightAcc returns cost function weight for quadratic acceleration term
      virtual double getWeightAcc() const =0;

      ///getWeightJerk returns cost function weight for quadratic jerk term
      virtual double getWeightJerk() const =0;
      
      ///getWeightEndPos returns cost function weight for quadratic position error term at end point
      virtual double getWeightEndPos() const =0;

      ///getWeightEndVel returns cost function weight for quadratic velocity error term at end point
      virtual double getWeightEndVel() const =0;

      ///getWeightEndAcc returns cost function weight for quadratic acceleration term at end point
      virtual double getWeightEndAcc() const =0;

      ///getSlackPos returns maximum slack of soft-constraints for position
      virtual double getSlackPos() const =0;

      ///getSlackVel returns maximum slack of soft-constraints for velocity
      virtual double getSlackVel() const =0;
      
      ///getSlackAcc returns maximum slack of soft-constraints for acceleration
      virtual double getSlackAcc() const =0;

      ///getAccLB returns lateral acceleration lower bound
      virtual double getAccLB() const=0;

      ///getAccUB returns lateral acceleration upper bound
      virtual double getAccUB() const=0;

      ///getJerkLB returns lateral jerk lower bound
      virtual double getJerkLB() const=0;

      ///getJerkLB returns lateral jerk upper bound
      virtual double getJerkUB() const=0;

      ///getCurvatureUB returns maximum curvature of path (relevant at low speeds)
      virtual double getCurvatureUB() const=0;

      ///getCurvatureLB returns minimum curvature of path (relevant at low speeds)
      virtual double getCurvatureLB() const=0;

      ///getRelativeHeadingUB returns upper bound on heading deviation from current lane's coordinate system
      virtual double getRelativeHeadingUB() const=0;

      ///getRelativeHeadingLB returns lower bound on heading deviation from current lane's coordinate system
      virtual double getRelativeHeadingLB() const=0;

      ///getMergeConstraintDelay returns a time-delay after which lateral position constraints are activated, if they are initially violated
      virtual double getMergeConstraintDelay() const=0;

      ///getHardSafetyDistanceToLaneBoundary returns the minimum distance between lane boundary and vehicle side
      virtual double getHardSafetyDistanceToLaneBoundary() const=0;

      ///getSoftSafetyDistanceToLaneBoundary returns the minimum distance between lane boundary and vehicle side, which is enforced, if sufficient space is available
      virtual double getSoftSafetyDistanceToLaneBoundary() const=0;

      ///getMinimumLateralControlSpace returns the minimum desired lateral control space: If vehicle has more space for lateral control, softSafetyDistanceToLaneBoundary is enforced
      virtual double getMinimumLateralControlSpace() const=0;

      ///getMaxCPUTime returns the maximum cpu time for one plan computation
      virtual double getMaxCPUTime() const=0;

      ///getLateralGridScale returns the size of a grid step d for lateral variations of a maneuver: maneuver variations will plan for offsets {...,-2d,-d,0,d,2d,..}
      virtual double getLateralGridScale() const=0;
      
    };
  }
}