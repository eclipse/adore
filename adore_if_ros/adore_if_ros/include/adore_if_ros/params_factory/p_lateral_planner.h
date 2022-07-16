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

#include <ros/ros.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore/params/ap_lateral_planner.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PLateralPlanner:public adore::params::APLateralPlanner,ROSParam
      {
        public:
        PLateralPlanner(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "lateral_planner/")
        {
        }
        //getWeightPos returns cost function weight for quadratic position error term
        virtual double getWeightPos() const override
        {
          double value = 6.0;
          static const std::string name = prefix_ + "weight_pos";
          get(name,value);
          return value;
        }
        //getWeightVel returns cost function weight for quadratic velocity error term
        virtual double getWeightVel() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "weight_vel";
          get(name,value);
          return value;
        }
        //getWeightAcc returns cost function weight for quadratic acceleration term
        virtual double getWeightAcc() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "weight_acc";
          get(name,value);
          return value;
        }
        //getWeightJerk returns cost function weight for quadratic jerk term
        virtual double getWeightJerk() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "weight_jerk";
          get(name,value);
          return value;
        }
        //getWeightEndPos returns cost function weight for quadratic position error term at end point
        virtual double getWeightEndPos() const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "weight_end_pos";
          get(name,value);
          return value;
        }
        //getWeightEndVel returns cost function weight for quadratic velocity error term at end point
        virtual double getWeightEndVel() const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "weight_end_vel";
          get(name,value);
          return value;
        }
        //getWeightEndAcc returns cost function weight for quadratic acceleration term at end point
        virtual double getWeightEndAcc() const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "weight_end_acc";
          get(name,value);
          return value;
        }
        //getSlackPos returns maximum slack of soft-constraints for position
        virtual double getSlackPos() const override
        {
          double value = 0.1;
          static const std::string name = prefix_ + "slack_pos";
          get(name,value);
          return value;
        }
        //getSlackVel returns maximum slack of soft-constraints for velocity
        virtual double getSlackVel() const override
        {
          double value = 0.1;
          static const std::string name = prefix_ + "slack_vel";
          get(name,value);
          return value;
        }
        //getSlackAcc returns maximum slack of soft-constraints for acceleration
        virtual double getSlackAcc() const override
        {
          double value = 0.1;
          static const std::string name = prefix_ + "slack_acc";
          get(name,value);
          return value;
        }
        //getAccLB returns longitudinal acceleration lower bound
        virtual double getAccLB() const override
        {
          double value = -10.0;
          static const std::string name = prefix_ + "acc_lb";
          get(name,value);
          return value;
        }
        //getAccUB returns longitudinal acceleration upper bound
        virtual double getAccUB() const override
        {
          double value = 10.0;
          static const std::string name = prefix_ + "acc_ub";
          get(name,value);
          return value;
        }
        //getJerkLB returns longitudinal jerk lower bound
        virtual double getJerkLB() const override
        {
          double value = -100.0;
          static const std::string name = prefix_ + "jerk_lb";
          get(name,value);
          return value;
        }
        //getJerkUB returns longitudinal jerk upper bound
        virtual double getJerkUB() const override
        {
          double value = 100.0;
          static const std::string name = prefix_ + "jerk_ub";
          get(name,value);
          return value;
        }
        //getCurvatureUB returns maximum curvature of path (relevant at low speeds)
        virtual double getCurvatureUB() const override
        {
          double value = 0.14;
          static const std::string name = prefix_ + "curvature_ub";
          get(name,value);
          return value;
        }
        //getCurvatureLB returns minimum curvature of path (relevant at low speeds)
        virtual double getCurvatureLB() const override
        {
          double value = -0.14;
          static const std::string name = prefix_ + "curvature_lb";
          get(name,value);
          return value;
        }
        //getRelativeHeadingUB returns upper bound on heading deviation from current lane's coordinate system
        virtual double getRelativeHeadingUB() const override
        {
          double value = 0.7;
          static const std::string name = prefix_ + "relative_heading_ub";
          get(name,value);
          return value;
        }
        //getRelativeHeadingLB returns lower bound on heading deviation from current lane's coordinate system
        virtual double getRelativeHeadingLB() const override
        {
          double value = -0.7;
          static const std::string name = prefix_ + "relative_heading_lb";
          get(name,value);
          return value;
        }
        //getMergeConstraintDelay returns a time-delay after which lateral position constraints are activated, if they are initially violated
        virtual double getMergeConstraintDelay() const override
        {
          double value = 3.0;
          static const std::string name = prefix_ + "merge_constraint_delay";
          get(name,value);
          return value;
        }

        ///getHardSafetyDistanceToLaneBoundary returns the minimum distance between lane boundary and vehicle side
        virtual double getHardSafetyDistanceToLaneBoundary() const override
        {
          double value = 0.2;
          static const std::string name = prefix_ + "hard_safety_distance_to_lane_boundary";
          get(name,value);
          return value;
        }

        ///getSoftSafetyDistanceToLaneBoundary returns the minimum distance between lane boundary and vehicle side, which is enforced, if sufficient space is available
        virtual double getSoftSafetyDistanceToLaneBoundary() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "soft_safety_distance_to_lane_boundary";
          get(name,value);
          return value;
        }

        ///getMinimumLateralControlSpace returns the minimum desired lateral control space: If vehicle has more space for lateral control, softSafetyDistanceToLaneBoundary is enforced
        virtual double getMinimumLateralControlSpace() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "minimum_lateral_control_space";
          get(name,value);
          return value;
        }

        //getMaxCPUTime returns the maximum cpu time for one plan computation
        virtual double getMaxCPUTime() const override
        {
          double value = 0.05;
          static const std::string name = prefix_ + "max_cpu_time";
          get(name,value);
          return value;
        }
        ///getLateralGridScale returns the size of a grid step d for lateral variations of a maneuver: maneuver variations will plan for offsets {...,-2d,-d,0,d,2d,..}
        virtual double getLateralGridScale() const override
        {
          double value = 0.2;
          static const std::string name = prefix_ + "lateral_grid_scale";
          get(name,value);
          return value;
        }

      };
    }
  }
}