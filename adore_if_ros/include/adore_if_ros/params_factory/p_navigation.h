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
#include <adore/params/ap_navigation.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PNavigation:public adore::params::APNavigation,ROSParam
      {
        public:
        PNavigation(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "navigation/")
        {
        }
        virtual bool getActivePlottingLocal() override
        {
          bool value = false;
          static const std::string name = prefix_ + "active_plotting_local";
          get(name,value);
          return value;
        }
		    virtual bool getActivePlottingGlobal() override
        {
          bool value = false;
          static const std::string name = prefix_ + "active_plotting_global";
          get(name,value);
          return value;
        }
		    virtual bool useScenarioManagerMap() override
        {
          bool value = false;
          static const std::string name = prefix_ + "use_scenario_mananger_map";
          get(name,value);
          return value;
        }
      };
    }
  }
}