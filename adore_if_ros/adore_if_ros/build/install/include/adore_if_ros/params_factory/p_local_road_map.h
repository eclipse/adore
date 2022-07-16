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
#include <adore/params/ap_local_road_map.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PLocalRoadMap:public adore::params::APLocalRoadMap,ROSParam
      {
        public:
        PLocalRoadMap(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "local_road_map/")
        {
        }
        virtual double getDiscardRadius()const override
        {
          double value = 200.0;
          static const std::string name = prefix_ + "discard_radius";
          get(name,value);
          return value;
        }
        virtual bool isNavigationActive()const override
        {
          bool value = false;
          static const std::string name = prefix_ + "navigation_active";
          get(name,value);
          return value;
        }
        virtual double getBorderTraceLength()const override
        {
          double value = 50.0;
          static const std::string name = prefix_ + "border_trace_length";
          get(name,value);
          return value;
        }
      };
    }
  }
}