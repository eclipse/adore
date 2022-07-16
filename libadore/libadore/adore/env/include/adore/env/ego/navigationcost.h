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
namespace adore
{
  namespace env
  {
    /**
     * @brief Struct to organize navigation cost
     * 
     */
    struct NavigationCost
    {
      private:
      double distanceToGoal_; /**< distance to goal */
      public:
      static double maximum_cost()
      {
        return 1.0e99;
      }
      bool operator<(const NavigationCost& other) const
      {
        return this->distanceToGoal_<other.distanceToGoal_;
      }
      bool operator==(const NavigationCost& other) const
      {
        return this->distanceToGoal_==other.distanceToGoal_;
      }
      /**
       * @brief Construct a new NavigationCost object
       * 
       */
      NavigationCost():distanceToGoal_(0.0){}
      /**
       * @brief Construct a new NavigationCost object
       * 
       * @param distanceToGoal distance to goal for the new NavigationCost object
       */
      NavigationCost(double distanceToGoal):distanceToGoal_(distanceToGoal){}
      /**
       * @brief Get the distance to goal
       * 
       * @return double distance to goal
       */
      double getDistanceToGoal()
      {
        return distanceToGoal_;
      }
      /**
       * @brief Set the distance to goal
       * 
       * @param value distance to goal
       */
      void setDistanceToGoal(double value)
      {
        distanceToGoal_ = value;
      }
      /**
       * @brief Get combined cost
       * 
       * @return double combined cost
       */
      double getCombinedCost()
      {
        return distanceToGoal_;
      }
    };
  }
}