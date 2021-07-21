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
#include <adore/params/ap_cooperation.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PCooperation:public adore::params::APCooperation,ROSParam
      {
        public:
        PCooperation(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "cooperation/")
        {
        }
        virtual int getCooperationMode()const override
        {
          int value = 1;
          static const std::string name = prefix_ + "cooperation_mode";
          get(name,value);
          return value;
        }
		    virtual double getAssumedChaseAcceleration()const override
        {
          double value = -1.5;
          static const std::string name = prefix_ + "assumed_chase_acceleration";
          get(name,value);
          return value;
        }
		    virtual double getNegotiationTime()const override
        {
          double value = 0.5;
          static const std::string name = prefix_ + "negotiation_time";
          get(name,value);
          return value;
        }
		    virtual double getAbsTimeUncertaintyForLC()const override
        {
          double value = 1.0;
          static const std::string name = prefix_ + "abs_time_uncertainly_for_lc";
          get(name,value);
          return value;
        }
		    virtual double getAbsPositionUncertainty()const override
        {
          double value = 4.0;
          static const std::string name = prefix_ + "abs_position_uncertainty";
          get(name,value);
          return value;
        }
		    virtual double getAbsVelocityUncertainty()const override
        {
          double value = 2.5;
          static const std::string name = prefix_ + "abs_velocity_uncertainty";
          get(name,value);
          return value;
        }
		    virtual int getSendRepetitiveMessages()const override
        {
          int value = 0;
          static const std::string name = prefix_ + "send_repetitive_messages";
          get(name,value);
          return value;
        }
        virtual int getUTMZone()const override
        {
          int value = 32;
          static const std::string name = prefix_ + "utm_zone";
          get(name,value);
          return value;
        }
      };
    }
  }
}