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
#include <adore/params/ap_mission_control.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PMissionControl:public adore::params::APMissionControl,ROSParam
      {
        public:
        PMissionControl(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "mission_control/")
        {
        }
        virtual double getDistanceToGoalForStop()const override
        {
          double value = 4.0;
          static const std::string name = prefix_ + "distance_to_goal_for_stop";
          get(name,value);
          return value;
        }
        virtual double getDistanceToGoalForStart()const override
        {
          double value = 50.0;
          static const std::string name = prefix_ + "distance_to_goal_for_start";
          get(name,value);
          return value;
        }
      };
    }
  }
}