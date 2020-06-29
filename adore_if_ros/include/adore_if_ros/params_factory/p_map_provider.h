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
#include <adore/params/ap_map_provider.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PMapProvider:public adore::params::APMapProvider,ROSParam
      {
        public:
        PMapProvider(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "map_provider/")
        {
        }
        //visibility radius of the map provider
        virtual double getVisibiltyRadius()const override
        {
          double value = 200;
          static const std::string name = prefix_ + "r";
          get(name,value);
          return value;
        }
        virtual bool getActivatePlotting()const override
        {
          bool value = false;
          static const std::string name = prefix_ + "activate_plotting";
          get(name,value);
          return value;
        }
        virtual bool getPlotCompleteMapInLocalView()const override
        {
          bool value = false;
          static const std::string name = prefix_ + "plot_complete_map_in_local_view";
          get(name,value);
          return value;
        }
        virtual bool useScenarioManagerMap()const override
        {
          bool value = false;
          static const std::string name = prefix_ + "use_scenario_manager_map";
          get(name,value);
          return value;
        }
      };
    }
  }
}