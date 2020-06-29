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
#include <adore/params/ap_longitudinal_planner.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PLongitudinalPlanner:public adore::params::APLongitudinalPlanner,ROSParam
      {
        public:
        PLongitudinalPlanner(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "longitudinal_planner/")
        {
        }
        //getWeightPos returns cost function weight for quadratic position error term
        virtual double getWeightPos() const override
        {
          double value = 2.0;
          static const std::string name = prefix_ + "weight_pos";
          get(name,value);
          return value;
        }
        //getWeightVel returns cost function weight for quadratic velocity error term
        virtual double getWeightVel() const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "weight_vel";
          get(name,value);
          return value;
        }
        //getWeightAcc returns cost function weight for quadratic acceleration term
        virtual double getWeightAcc() const override
        {
          double value = 5.0;
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
          double value = 0.5;
          static const std::string name = prefix_ + "slack_pos";
          get(name,value);
          return value;
        }
        //getSlackVel returns maximum slack of soft-constraints for velocity
        virtual double getSlackVel() const override
        {
          double value = 0.01;
          static const std::string name = prefix_ + "slack_vel";
          get(name,value);
          return value;
        }
        //getSlackAcc returns maximum slack of soft-constraints for acceleration
        virtual double getSlackAcc() const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "slack_acc";
          get(name,value);
          return value;
        }
        //getAccLB returns longitudinal acceleration lower bound
        virtual double getAccLB() const override
        {
          double value = -3.0;
          static const std::string name = prefix_ + "acc_lb";
          get(name,value);
          return value;
        }
        //getAccUB returns longitudinal acceleration upper bound
        virtual double getAccUB() const override
        {
          double value = 2.0;
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
        //getJerkLB returns longitudinal jerk upper bound
        virtual double getJerkUB() const override
        {
          double value = 100.0;
          static const std::string name = prefix_ + "jerk_ub";
          get(name,value);
          return value;
        }
        //getAccLatUB returns the absolute lateral acceleration bound which has to be maintained by reducing speed
        virtual double getAccLatUB() const override
        {
          double value = 2.0;
          static const std::string name = prefix_ + "acc_lat_ub";
          get(name,value);
          return value;
        }
        //getAccLatUB_minVelocity returns the minimum velocity, which is always feasible despite getAccLatUB
        virtual double getAccLatUB_minVelocity() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "acc_lat_ub_min_velocity";
          get(name,value);
          return value;
        }
        //getConstraintClearancePos returns the offset of the reference from the position constraints
        virtual double getConstraintClearancePos() const override
        {
          double value = 0.1;
          static const std::string name = prefix_ + "constraint_clearance_pos";
          get(name,value);
          return value;
        }
        //getConstraintClearanceVel returns the offset of the reference from the velocity constraints
        virtual double getConstraintClearanceVel() const override
        {
          double value = 0.05;
          static const std::string name = prefix_ + "constraint_clearance_vel";
          get(name,value);
          return value;
        }


        ///getMinWidthStop returns the minimum lane width, below/at which vehicle stops: Should be greater or equal to ap_vehicle::bodyWidth + 2*ap_lateral_planner_::HardSafetyDistanceToLaneBoundary
        virtual double getMinWidthStop() const override
        {
          double value = 2.2;
          static const std::string name = prefix_ + "min_width_stop";
          get(name,value);
          return value;
        }

        ///getMinWidthSlow returns the minimum lane width, below/at which vehicle moves slowly: Should be greater or queal to minWidthStop
        virtual double getMinWidthSlow() const override
        {
          double value = 2.5;
          static const std::string name = prefix_ + "min_width_slow";
          get(name,value);
          return value;
        }

        ///getMinWidthSlowSpeed returns the slow speed to be applied, if lane width equals minWidthSlow: Should be greater than 0
        virtual double getMinWidthSlowSpeed() const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "min_width_slow_speed";
          get(name,value);
          return value;
        }

        ///getMinWidthFast returns the minimum lane width, below/at which vehicle moves fast: Should be greater or queal to minWidthSlow
        virtual double getMinWidthFast() const override
        {
          double value = 3.7;
          static const std::string name = prefix_ + "min_width_fast";
          get(name,value);
          return value;
        }

        ///getMinWidthFastSpeed returns the fast speed to be applied, if lane width equals minWidthFast: Should be greater than minWidthSlowSpeed
        virtual double getMinWidthFastSpeed() const override
        {
          double value = 40.0/3.6;
          static const std::string name = prefix_ + "min_width_fast_speed";
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
      };
    }
  }
}