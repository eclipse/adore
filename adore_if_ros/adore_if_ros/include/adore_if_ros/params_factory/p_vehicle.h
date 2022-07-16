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
#include <adore/params/ap_vehicle.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PVehicle:public adore::params::APVehicle, ROSParam
        {
          public:
          PVehicle(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "Vehicle/")
          {
          }
          //ID of current vehicle
          virtual std::string get_vehicle_id()const override
          {
            std::string value = "default vehicle";
            static const std::string name = prefix_ + "VehicleID";
            get(name,value);
            return value;
          }
          //front axle to cog
          virtual double get_a()const override
          {
            double value = 1.014;
            static const std::string name = prefix_ + "a";
            get(name,value);
            return value;
          }
          //rear axle to cog
          virtual double get_b()const override
          {
            double value = 1.676;
            static const std::string name = prefix_ + "b";
            get(name,value);
            return value;
          }
          //front axle to front border
          virtual double get_c()const override
          {
            double value = 0.97;
            static const std::string name = prefix_ + "c";
            get(name,value);
            return value;
          }
          //rear border to rear axle
          virtual double get_d()const override
          {
            double value = 1.12;
            static const std::string name = prefix_ + "d";
            get(name,value);
            return value;
          }
          //mass
          virtual double get_m()const override
          {
            double value = 1800;
            static const std::string name = prefix_ + "m";
            get(name,value);
            return value;
          }
          //friction coefficient
          virtual double get_mu()const override
          {
            double value = 0.8;
            static const std::string name = prefix_ + "mu";
            get(name,value);
            return value;
          }
          //gravitational constant
          virtual double get_g()const override
          {
            double value = 9.81;
            static const std::string name = prefix_ + "g";
            get(name,value);
            return value;
          }
          //cog height above ground
          virtual double get_h()const override
          {
            double value = 0.5;
            static const std::string name = prefix_ + "h";
            get(name,value);
            return value;
          }
          //front normalized tire stiffness for bicycle model
          virtual double get_cf()const override
          {
            double value = 10.8;
            static const std::string name = prefix_ + "cf";
            get(name,value);
            return value;
          }
          //rear normalized tire stiffness for bicycle model
          virtual double get_cr()const override
          {
            double value = 17.8;
            static const std::string name = prefix_ + "cr";
            get(name,value);
            return value;
          }
          //rotational inertia around up axis devided by mass
          virtual double get_Iz_m()const override
          {
            double value = 1.57;
            static const std::string name = prefix_ + "Iz_m";
            get(name,value);
            return value;
          }
          //track width front
          virtual double get_wf()const override
          {
            double value = 1.7;
            static const std::string name = prefix_ + "wf";
            get(name,value);
            return value;
          }
          //track width rear
          virtual double get_wr()const override
          {
            double value = 1.7;
            static const std::string name = prefix_ + "wr";
            get(name,value);
            return value;
          }
          //maximum width of body, usually between corners of mirrors
          virtual double get_bodyWidth()const override
          {
            double value = 1.82;
            static const std::string name = prefix_ + "bodyWidth";
            get(name,value);
            return value;
          }
          //steering ratio
          virtual double get_steeringRatio()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "steeringRatio";
            get(name,value);
            return value;
          }
          virtual double get_steeringAngleOffsetMeasured()const override
          {
            double value = -0.028;
            static const std::string name = prefix_ + "steeringAngleOffsetMeasured";
            get(name,value);
            return value;
          }
          virtual double get_steeringAngleOffsetCommand()const override
          {
            double value = -0.028;
            static const std::string name = prefix_ + "steeringAngleOffsetCommand";
            get(name,value);
            return value;
          }
          virtual double get_steeringAngleMax()const override
          {
            double value = 1.57;
            static const std::string name = prefix_ + "steeringAngleMax";
            get(name,value);
            return value;
          }
          virtual double get_steeringAngleMin()const override
          {
            double value = -1.57;
            static const std::string name = prefix_ + "steeringAngleMin";
            get(name,value);
            return value;
          }
          //unnormalized cornering stiffness
          virtual double get_C()const override
          {
            double value = 63000.0;
            static const std::string name = prefix_ + "C";
            get(name,value);
            return value;          
          }
          //returns the percentage of brake force allocated to the front axle, e.g. 0.6 is a typical value
          virtual double get_brakeBalanceFront()const override
          {
            double value = 0.6;
            static const std::string name = prefix_ + "brakeBalanceFront";
            get(name,value);
            return value;
          }
          //returns the percentage of acceleration force allocated to the front axle, e.g. 1.0 for front drive
          virtual double get_accelerationBalanceFront()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "accelerationBalanceFront";
            get(name,value);
            return value;
          }
          virtual double get_observationPointForPosition()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "observationPointForPosition";
            get(name,value);
            return value;
          }
          virtual double get_observationPointForVelocity()const override
          {
            double value = 1.676;
            static const std::string name = prefix_ + "observationPointForVelocity";
            get(name,value);
            return value;
          }
          virtual double get_observationPointForAcceleration()const override
          {
            double value = 0.0;
            static const std::string name = prefix_ + "observationPointForAcceleration";
            get(name,value);
            return value;
          }
          //returns the flag of currently used vehicle
          virtual double get_vehicleFlag()const override
          {
            return 0.0;
          }
        };
    }
  }
}