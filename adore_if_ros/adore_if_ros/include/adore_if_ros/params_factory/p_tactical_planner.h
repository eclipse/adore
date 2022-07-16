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
#include <adore/params/ap_tactical_planner.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PTacticalPlanner:public adore::params::APTacticalPlanner,ROSParam
      {
        public:
        PTacticalPlanner(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "tactical_planner/")
        {
        }
        virtual double getGlobalSpeedLimit()const override
        {
          double value = 10.0;
          static const std::string name = prefix_ + "global_speed_limit";
          get(name,value);
          return value;
        }
		    virtual double getResetRadius()const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "reset_radius";
          get(name,value);
          return value;
        }
		    virtual double getAccLatUB()const override
        {
          double value = 2.0;
          static const std::string name = prefix_ + "acc_lat_ub";
          get(name,value);
          return value;
        }
		    virtual double getAccLatUB_minVelocity()const override
        {
          double value = 3.0;
          static const std::string name = prefix_ + "acc_lat_ub_min_vel";
          get(name,value);
          return value;
        }
		    virtual double getAccLonUB()const override
        {
          double value = 2.0;
          static const std::string name = prefix_ + "acc_lon_ub";
          get(name,value);
          return value;
        }
		    virtual double getAccLonLB()const override
        {
          double value = -3.0;
          static const std::string name = prefix_ + "acc_lon_lb";
          get(name,value);
          return value;
        }
		    virtual double getFrontTimeGap()const override
        {
          double value = 0.9;
          static const std::string name = prefix_ + "front_time_gap";
          get(name,value);
          return value;
        }
		    virtual double getRearTimeGap()const override
        {
          double value = 0.9;
          static const std::string name = prefix_ + "rear_time_gap";
          get(name,value);
          return value;
        }
		    virtual double getFrontSGap()const override
        {
          double value = 6.0;
          static const std::string name = prefix_ + "front_s_gap";
          get(name,value);
          return value;
        }
        virtual double getLowerBoundFrontSGapForLF()const override
        {
          double value = 7.0;
          static const std::string name = prefix_ + "lower_bound_lf_front_s_gap";
          get(name,value);
          return value;
        }
		    virtual double getRearSGap()const override
        {
          double value = 2.0;
          static const std::string name = prefix_ + "rear_s_gap";
          get(name,value);
          return value;
        }
		    virtual double getChaseReferenceOffset()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "chase_reference_offset";
          get(name,value);
          return value;
        }
		    virtual double getLeadReferenceOffset()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "lead_reference_offset";
          get(name,value);
          return value;
        }
		    virtual double getFrontReferenceOffset()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "front_reference_offset";
          get(name,value);
          return value;
        }
		    virtual double getGapAlignment()const override
        {
          double value = 0.5;
          static const std::string name = prefix_ + "gap_alignment";
          get(name,value);
          return value;
        }
        virtual double getAssumedNominalAccelerationMinimum()const override
        {
          double value = -1.0;
          static const std::string name = prefix_ + "assumed_nominal_acceleration_minimum";
          get(name,value);
          return value;
        }
        virtual double getMaxNavcostLoss()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "max_navcost_loss";
          get(name,value);
          return value;
        }
  			virtual bool getEnforceMonotonousNavigationCost()const override
        {
          bool value = false;
          static const std::string name = prefix_ + "enforce_monotonous_navigation_cost";
          get(name,value);
          return value;
        }
        virtual double getTimeoutForPreferredLCAfterManuallySetIndicator()const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "preferred_lc_by_manual_indicator_timeout";
          get(name,value);
          return value;
        }
        virtual double getLVResetVelocity()const override
        {
          double value = 1.5;
          static const std::string name = prefix_ + "lane_view_reset_velocity";
          get(name,value);
          return value;
        }
        virtual double getTimeoutForLangechangeSuppression()const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "lanechange_suppression_timeout";
          get(name,value);
          return value;
        }
        virtual double getCollisionDetectionFrontBufferSpace()const override
        {
          double value = 4.0;
          static const std::string name = prefix_ + "collision_detection_front_buffer_space";
          get(name,value);
          return value;
        }
        virtual double getCollisionDetectionLateralPrecision()const override
        {
          double value = 0.05;
          static const std::string name = prefix_ + "collision_detection_lateral_precision";
          get(name,value);
          return value;
        }
        virtual double getCollisionDetectionLateralError()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "collision_detection_lateral_error";
          get(name,value);
          return value;
        }
        virtual double getCollisionDetectionLongitudinalError()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "collision_detection_longitudinal_error";
          get(name,value);
          return value;
        }
			  virtual double getNominalSwathAccelerationError()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "nominal_swath_acceleration_error";
          get(name,value);
          return value;
        }
  			/// getCoercionPreventionStrategy returns 0 switched off, 1 objective function, 2 constraint
        virtual int getCoercionPreventionStrategy()const override
        {
          int value = 0;
          static const std::string name = prefix_ + "coercion_prevention_strategy";
          get(name,value);
          return value;
        }
        virtual double getIndicatorLookahead()const override
        {
          double value = 50.0;
          static const std::string name = prefix_ + "indicator_lookahead";
          get(name,value);
          return value;
        }
        virtual double getHorizonStopReferenceDistance()const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "horizon_stop_reference_distance";
          get(name,value);
          return value;
        }
        virtual double getTerminateAfterFirstStopThresholdSpeed()const override
        {
          double value = 0.3;
          static const std::string name = prefix_ + "terminate_after_first_stop_threshold_speed";
          get(name,value);
          return value;
        }

      };
    }
  }
}