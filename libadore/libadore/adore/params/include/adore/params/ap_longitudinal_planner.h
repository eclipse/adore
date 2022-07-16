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
     * @brief abstract class containing parameters related to configuring the longitudinal planner
     * 
     */
    class APLongitudinalPlanner
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

      ///getAccLB returns longitudinal acceleration lower bound
      virtual double getAccLB() const=0;
      
      ///getAccUB returns longitudinal acceleration upper bound
      virtual double getAccUB() const=0;

      ///getAccUBSlow returns acceleration upper bound at low speeds
      virtual double getAccUBSlow() const=0;

      ///getVAccUBSlow returns speed up to which slow speed acceleration is used
      virtual double getVAccUBSlow() const=0;

      ///getAccLB returns longitudinal acceleration lower bound
      virtual double getComfortAccLB() const=0;
      
      ///getAccUB returns longitudinal acceleration upper bound
      virtual double getComfortAccUB() const=0;

      ///getJerkLB returns longitudinal jerk lower bound
      virtual double getJerkLB() const=0;

      ///getJerkLB returns longitudinal jerk upper bound
      virtual double getJerkUB() const=0;
      
      ///getAccLatUB returns the absolute lateral acceleration bound which has to be maintained by reducing speed
      virtual double getAccLatUB() const=0;

      ///getAccLatUB_minVelocity returns the minimum velocity, which is always feasible despite getAccLatUB
      virtual double getAccLatUB_minVelocity() const=0;

      ///getConstraintClearancePos returns the longitudinal offset of the reference from the position constraints
      virtual double getConstraintClearancePos() const=0;

      ///getConstraintClearanceVel returns the offset of the reference from the velocity constraints
      virtual double getConstraintClearanceVel() const=0;

      ///getMinWidthStop returns the minimum lane width, below/at which vehicle stops: Should be greater or equal to ap_vehicle::bodyWidth + 2*ap_lateral_planner_::HardSafetyDistanceToLaneBoundary
      virtual double getMinWidthStop() const=0;

      ///getMinWidthSlow returns the minimum lane width, below/at which vehicle moves slowly: Should be greater or queal to minWidthStop
      virtual double getMinWidthSlow() const=0;

      ///getMinWidthSlowSpeed returns the slow speed to be applied, if lane width equals minWidthSlow: Should be greater than 0
      virtual double getMinWidthSlowSpeed() const=0;

      ///getMinWidthFast returns the minimum lane width, below/at which vehicle moves fast: Should be greater or queal to minWidthSlow
      virtual double getMinWidthFast() const=0;

      ///getMinWidthFastSpeed returns the fast speed to be applied, if lane width equals minWidthFast: Should be greater than minWidthSlowSpeed
      virtual double getMinWidthFastSpeed() const=0;

      ///getMaxCPUTime returns the maximum cpu time for one plan computation
      virtual double getMaxCPUTime() const=0;

      ///determin stop mode for red lights: true - always before red light, continue driving after; false - based on deceleration curve
      virtual bool stopAtRedLights_always_before()const=0;

      ///stopAtRedLights_max_connection_length returns the maximum length for which clearance based on tcd state is considered - after maximum length, vehicle continues. 
      virtual double stopAtRedLights_max_connection_length() const=0;
			/**
			 * @brief distance between stop position and conflict point
			 */
			virtual double getStopDistanceToConflictPoint()const=0;
    };
  }
}