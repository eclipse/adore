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
#include <adore/params/ap_trajectory_tracking.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PTrajectoryTracking:public adore::params::APTrajectoryTracking,ROSParam
      {
        public:
          PTrajectoryTracking(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "trajectory_tracking/")
          {
          }
          //lateral control gain for lateral error ey
          virtual double getKey()const override
          {
            double value = 0.05;
            static const std::string name = prefix_ + "key";
            get(name,value);
            return value;
          }
          //lateral control gain for yaw angle error epsi
          virtual double getKepsi()const override
          {
            double value = 0.4;
            static const std::string name = prefix_ + "kepsi";
            get(name,value);
            return value;
          }
          //lateral control gain for yaw rate error eomega
          virtual double getKeomega()const override
          {
            double value = 0.05;
            static const std::string name = prefix_ + "keomega";
            get(name,value);
            return value;
          }
          //returns I control gain for lateral direction
          virtual double getKIy()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kIy";
            get(name,value);
            return value;
          }
          //returns I control gain for longitudinal direction
          virtual double getKIx()const override
          {
            double value = 0.1;
            static const std::string name = prefix_ + "kIx";
            get(name,value);
            return value;
          }
          //returns P control gain for longitudinal direction
          virtual double getK0x()const override
          {
            double value = 0.9;
            static const std::string name = prefix_ + "k0x";
            get(name,value);
            return value;
          }
          //returns D control gain for longitudinal direction
          virtual double getK1x()const override
          {
            double value = 1.6;
            static const std::string name = prefix_ + "k1x";
            get(name,value);
            return value;
          }
          //returns factor for maximum tire force requestable by controller, |f_requested|<muCtrlMax * f_max
          virtual double getMuCtrlMax()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "muCtrlMax";
            get(name,value);
            return value;
          }
          //hard coded maximum longitudinal acceleration
          virtual double getAxMax()const override
          {
            double value = 2.0;
            static const std::string name = prefix_ + "axMax";
            get(name,value);
            return value;
          }
          //hard coded minimum longitudinal acceleration
          virtual double getAxMin()const override
          {
            double value = -3.0;
            static const std::string name = prefix_ + "axMin";
            get(name,value);
            return value;
          }
          //static trajectory tracking offset in longitudinal direction, which should be compensated by tracking controller
          virtual double getExStatic()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "exStatic";
            get(name,value);
            return value;
          }
          //static trajectory tracking offset in lateral direction, which should be compensated by tracking controller
          virtual double getEyStatic()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "eyStatic";
            get(name,value);
            return value;
          }
          //the maximum controllable steering angle
          virtual double getDeltaMax()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "deltaMax";
            get(name,value);
            return value;
          }
          //the minimum controllable steering angle
          virtual double getDeltaMin()const override
          {
            double value = -1.0;
            static const std::string name = prefix_ + "deltaMin";
            get(name,value);
            return value;
          }
          //steering angle: maximum absolute control input change per control update. Maximum steering rate then depends on execution rate of controller.
          virtual double getDDeltaMax()const override
          {
            double value = 0.8;
            static const std::string name = prefix_ + "dDeltaMax";
            get(name,value);
            return value;
          }
          //returns gain for braking torque calculation
          virtual double getBrakingTorqueGain()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "brakingTorqueGain";
            get(name,value);
            return value;
          }
          //returns maxium braking torque rate
          virtual double getDBrakingTorqueMax()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "dBrakingTorqueMax";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for speed error (P)
          virtual double getKPev_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kPev_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for integrated speed error (I)
          virtual double getKIev_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kIev_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for x error (P)
          virtual double getKPex_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kPex_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for integrated x error (I)
          virtual double getKIex_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kIex_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for y error (P)
          virtual double getKPey_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kPey_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for psi error (P)
          virtual double getKPepsi_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kPepsi_r";
            get(name,value);
            return value;
          }
          //reverse controller: control gain for integrated psi error (I)
          virtual double getKIepsi_r()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "kIepsi_r";
            get(name,value);
            return value;
          }
          //gain for steering rate limiter
          virtual double getSteeringRateLimiterGain()const override
          {
            double value = 1.3;
            static const std::string name = prefix_ + "steeringRateLimiterGain";
            get(name,value);
            return value;
          }
      };
    }
  }
}