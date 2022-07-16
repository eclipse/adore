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
#include <adore/params/ap_localizationmodel.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PLocalizationModel:public adore::params::APLocalizationModel, ROSParam
        {
          public:
          PLocalizationModel(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "LocalizationModel/")
          {
          }
			virtual double get_drift_rate_pos()const override
            {
            double value = 0.0;
            static const std::string name = prefix_ + "drift_rate_pos";
            get(name,value);
            return value;
            }
			virtual double get_drift_deviation_pos()const override
            {
            double value = 0.0;
            static const std::string name = prefix_ + "drift_deviation_pos";
            get(name,value);
            return value;
            }
            virtual double get_jump_deviation_pos()const override 
            {
            double value = 0.0;
            static const std::string name = prefix_ + "jump_deviation_pos";
            get(name,value);
            return value;
            }
            virtual double get_jump_threshold_pos()const override
            {
            double value = 0.0;
            static const std::string name = prefix_ + "jump_threshold_pos";
            get(name,value);
            return value;
            }
            virtual double get_jump_deviation_heading()const override
            {
            double value = 0.0;
            static const std::string name = prefix_ + "jump_deviation_heading";
            get(name,value);
            return value;                
            }
            virtual double get_jump_threshold_heading()const override
            {
            double value = 0.0;
            static const std::string name = prefix_ + "jump_threshold_heading";
            get(name,value);
            return value;                
            }
        
        };
    }
  }
}