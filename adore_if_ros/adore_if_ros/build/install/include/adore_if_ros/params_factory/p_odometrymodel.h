/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore/params/ap_odometrymodel.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class POdometryModel:public adore::params::APOdometryModel, ROSParam
        {
          public:
          POdometryModel(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "OdometryModel/")
          {
          }
          virtual double get_k_e_vx()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "k_e_vx";
            get(name,value);
            return value;
          }
          virtual double get_k_e_vy()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "k_e_vy";
            get(name,value);
            return value;
          }
          virtual double get_k_e_omega()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "k_e_omega";
            get(name,value);
            return value;
          }
          virtual double get_k_e_ax()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "k_e_ax";
            get(name,value);
            return value;
          }
        
        };
    }
  }
}