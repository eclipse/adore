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
#include <adore/params/ap_trajectory_generation.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PTrajectoryGeneration:public adore::params::APTrajectoryGeneration,ROSParam
      {
        public:
        PTrajectoryGeneration(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "trajectory_generation/")
        {
        }
        //cor to planning point: movement of planning point shall planned by the trajectory planner
        virtual double get_rho()const override
        {
          double value = 2.2;
          static const std::string name = prefix_ + "rho";
          get(name,value);
          return value;
        }
        //zero dynamics integration length
        virtual double getZDIntegrationLength()const override
        {
          double value = 3.0;
          static const std::string name = prefix_ + "zd_integration_length";
          get(name,value);
          return value;
        }
        //zero dynamics step size
        virtual double getZDIntegrationStep()const override
        {
          double value = 0.01;
          static const std::string name = prefix_ + "zd_integration_step";
          get(name,value);
          return value;
        }
        //number of set points in set-point request
        virtual int getSetPointCount()const override
        {
          int value = 100;
          static const std::string name = prefix_ + "set_point_count";
          get(name,value);
          return value;
        }
        ///time after which emergency maneuver kicks in
        virtual double getEmergencyManeuverDelay()const override
        {
          double value = 0.5;
          static const std::string name = prefix_ + "emergency_maneuver_delay";
          get(name,value);
          return value;
        }

      };
    }
  }
}