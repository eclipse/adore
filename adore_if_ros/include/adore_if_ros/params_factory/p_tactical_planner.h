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
      };
    }
  }
}