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
#include <adore/params/ap_checkpoints.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PCheckpoints:public adore::params::APCheckpoints,ROSParam
      {
        public:
        PCheckpoints(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "checkpoints/")
        {
        }
        virtual double getClearDistance()const override
        {
          double value = 20.0;
          static const std::string name = prefix_ + "clear_distance";
          get(name,value);
          return value;
        }
		    virtual double getClearTimeout()const override
        {
          double value = 5.0;
          static const std::string name = prefix_ + "clear_timeout";
          get(name,value);
          return value;
        }
      };
    }
  }
}