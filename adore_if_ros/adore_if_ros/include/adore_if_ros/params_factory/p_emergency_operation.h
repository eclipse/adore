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
#include <adore/params/ap_emergency_operation.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PEmergencyOperation:public adore::params::APEmergencyOperation,ROSParam
      {
        public:
          PEmergencyOperation(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "emergency_operation/")
          {
          }
          //lateral path tracking gain for terminal maneuver
          virtual double getKy()const override
          {
            double value = 0.05;
            static const std::string name = prefix_ + "kyTerm";
            get(name,value);
            return value;
          }
          //lateral path tracking gain for terminal maneuver
          virtual double getKpsi()const override
          {
            double value = 0.4;
            static const std::string name = prefix_ + "kpsiTerm";
            get(name,value);
            return value;
          }
          //hard coded longitudinal minimum acceleration
          virtual double getamin()const override
          {
            double value = -3.0;
            static const std::string name = prefix_ + "axMinTerm";
            get(name,value);
            return value;
          }
          //hard coded maximum longitudinal acceleration
          virtual double getamax()const override
          {
            double value = 2.0;
            static const std::string name = prefix_ + "axMaxTerm";
            get(name,value);
            return value;
          }
          virtual double getDeltaMax()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "deltaMaxTerm";
            get(name,value);
            return value;
          }
          virtual double getDeltaMin()const override
          {
            double value = -1.0;
            static const std::string name = prefix_ + "deltaMinTerm";
            get(name,value);
            return value;
          }
          virtual double getEmergencyManeuverAMin()const override
          {
            double value = -3.0;
            static const std::string name = prefix_ + "emergencyManeuverAMin";
            get(name,value);
            return value;
          }
          virtual double getEmergencyManeuverAMax()const override
          {
            double value = -1.0;
            static const std::string name = prefix_ + "emergencyManeuverAMax";
            get(name,value);
            return value;
          }
          virtual double getEmergencyManeuverAStall()const override
          {
            double value = -2.0;
            static const std::string name = prefix_ + "emergencyManeuverAStall";
            get(name,value);
            return value;
          }
          virtual double getEmergencyManeuverTStall()const override
          {
            double value = 0.5;
            static const std::string name = prefix_ + "emergencyManeuverTStall";
            get(name,value);
            return value;
          }
          virtual double getEmergencyManeuverJMin()const override
          {
            double value = -10.0;
            static const std::string name = prefix_ + "emergencyManeuverJMin";
            get(name,value);
            return value;
          }
      };
    }
  }
}