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
#include <adore/params/ap_prediction.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PPrediction:public adore::params::APPrediction,ROSParam
      {
        public:
        PPrediction(ros::NodeHandle n,std::string prefix)
        :ROSParam(n,prefix + "prediction/")
        {
        }
        ///prediction duration for objects that can be matched to road
        virtual double get_roadbased_prediction_duration()const override 
        {
          double value = 5.0;
          static const std::string name = prefix_ + "roadbased_prediction_duration";
          get(name,value);
          return value;
        }
        ///maximum acceleration for normal behavior for objects that can be matched to road
        virtual double get_roadbased_expected_acc_ub()const override 
        {
          double value = 0.0;
          static const std::string name = prefix_ + "roadbased_expected_acc_ub";
          get(name,value);
          return value;
        }
        ///delay after which expected_acc_ub is applied
        virtual double get_roadbased_expected_acc_ub_delay()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "roadbased_expected_acc_ub_delay";
          get(name,value);
          return value;
        }
        ///minimum acceleration for normal behavior for objects that can be matched to road
        virtual double get_roadbased_expected_acc_lb()const override 
        {
          double value = -3.0;
          static const std::string name = prefix_ + "roadbased_expected_acc_lb";
          get(name,value);
          return value;
        }
        ///maximum velocity for normal behavior for objects that can be matched to road
        virtual double get_roadbased_expected_vel_ub()const override 
        {
          double value = 30.0;
          static const std::string name = prefix_ + "roadbased_expected_vel_ub";
          get(name,value);
          return value;
        }
        ///maximum acceleration for worst-case behavior for objects that can be matched to road
        virtual double get_roadbased_worstcase_acc_ub()const override 
        {
          double value = 3.0;
          static const std::string name = prefix_ + "roadbased_worstcase_acc_ub";
          get(name,value);
          return value;
        }
        ///delay after which worstcase_acc_ub is applied
        virtual double get_roadbased_worstcase_acc_ub_delay()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "roadbased_worstcase_acc_ub_delay";
          get(name,value);
          return value;
        }
        ///minimum acceleration for worst-case for objects that can be matched to road
        virtual double get_roadbased_worstcase_acc_lb()const override 
        {
          double value = -10.0;
          static const std::string name = prefix_ + "roadbased_worstcase_acc_lb";
          get(name,value);
          return value;
        }
        ///maximum velocity for worst-case for objects that can be matched to road
        virtual double get_roadbased_worstcase_vel_ub()const override 
        {
          double value = 30.0;
          static const std::string name = prefix_ + "roadbased_worstcase_vel_ub";
          get(name,value);
          return value;
        }
        /// maximum difference between object and road heading for object to be matchable to road
        virtual double get_roadbased_heading_deviation_ub()const override 
        {
          double value = M_PI*0.25;
          static const std::string name = prefix_ + "roadbased_heading_deviation_ub";
          get(name,value);
          return value;
        }
        /// precision of object shape approximation in lateral direction for objects that can be matched to road
        virtual double get_roadbased_lat_precision()const override 
        {
          double value = 0.2;
          static const std::string name = prefix_ + "roadbased_lat_precision";
          get(name,value);
          return value;
        }
        /// assumed maximum lateral detection error for objects that can be matched to road (buffer zone)
        virtual double get_roadbased_lat_error()const override 
        {
          double value = 0.1;
          static const std::string name = prefix_ + "roadbased_lat_error";
          get(name,value);
          return value;
        }
        /// assumed maximum longitudinal detection error for objects that can be matched to road (buffer zone)
        virtual double get_roadbased_lon_error()const override 
        {
          double value = 0.5;
          static const std::string name = prefix_ + "roadbased_lon_error";
          get(name,value);
          return value;
        }
        /// time buffer ahead of an object (objrect predicted to arrive given seconds earlier at a location)
        virtual double get_roadbased_time_headway()const override 
        {
          double value = 2.0;
          static const std::string name = prefix_ + "roadbased_time_headway";
          get(name,value);
          return value;
        }
        /// time buffer behind object (object predicted to leave a location given seconds later)
        virtual double get_roadbased_time_leeway()const override 
        {
          double value = 1.0;
          static const std::string name = prefix_ + "roadbased_time_leeway";
          get(name,value);
          return value;
        }
        /// prediction duration for objects that can not be matched to road
        virtual double get_offroad_prediction_duration()const override 
        {
          double value = 5.0;
          static const std::string name = prefix_ + "offroad_prediction_duration";
          get(name,value);
          return value;
        }
        /// maximum acceleration for normal behavior for objects that can not be matched to road
        virtual double get_offroad_expected_acc_ub()const override 
        {
          double value = 0.0;
          static const std::string name = prefix_ + "offroad_expected_acc_ub";
          get(name,value);
          return value;
        }
        /// minimum acceleration for normal behavior for objects that can not be matched to road
        virtual double get_offroad_expected_acc_lb()const override 
        {
          double value = -3.0;
          static const std::string name = prefix_ + "offroad_expected_acc_lb";
          get(name,value);
          return value;
        }
        /// maximum velocity for normal behavior for objects that can not be matched to road
        virtual double get_offroad_expected_vel_ub()const override 
        {
          double value = 30.0;
          static const std::string name = prefix_ + "offroad_expected_vel_ub";
          get(name,value);
          return value;
        }
        /// maximum acceleration for worst-case behavior for objects that can not be matched to road
        virtual double get_offroad_worstcase_acc_ub()const override 
        {
          double value = 3.0;
          static const std::string name = prefix_ + "offroad_worstcase_acc_ub";
          get(name,value);
          return value;
        }
        ///delay after which worstcase_acc_ub is applied
        virtual double get_offroad_worstcase_acc_ub_delay()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "offroad_worstcase_acc_ub_delay";
          get(name,value);
          return value;
        }
        /// minimum acceleration for worst-case behavior for objects that can not be matched to road
        virtual double get_offroad_worstcase_acc_lb()const override 
        {
          double value = -10.0;
          static const std::string name = prefix_ + "offroad_worstcase_acc_lb";
          get(name,value);
          return value;
        }
        /// maximum velocity for worst-case behavior for objects that can not be matched to road
        virtual double get_offroad_worstcase_vel_ub()const override 
        {
          double value = 30.0;
          static const std::string name = prefix_ + "offroad_worstcase_vel_ub";
          get(name,value);
          return value;
        }
        /// precision of object shape approximation in lateral direction for objects that can not be matched to road
        virtual double get_offroad_lat_precision()const override 
        {
          double value = 0.2;
          static const std::string name = prefix_ + "offroad_lat_precision";
          get(name,value);
          return value;
        }
        /// assumed maximum lateral detection error for objects that can not be matched to road
        virtual double get_offroad_lat_error()const override 
        {
          double value = 0.1;
          static const std::string name = prefix_ + "offroad_lat_error";
          get(name,value);
          return value;
        }
        /// assumed maximum longitudinal detection error for objects that can not be matched to road
        virtual double get_offroad_lon_error()const override 
        {
          double value = 0.5;
          static const std::string name = prefix_ + "offroad_lon_error";
          get(name,value);
          return value;
        }
        /// time buffer ahead of an object (objrect predicted to arrive given seconds earlier at a location)
        virtual double get_offroad_time_headway()const override 
        {
          double value = 2.0;
          static const std::string name = prefix_ + "offroad_time_headway";
          get(name,value);
          return value;
        }
        /// time buffer behind object (object predicted to leave a location given seconds later)
        virtual double get_offroad_time_leeway()const override 
        {
          double value = 1.0;
          static const std::string name = prefix_ + "offroad_time_leeway";
          get(name,value);
          return value;
        }
            /// distinction between clutter and static traffic objects: how far into road has object to extend to be recognized as traffic?
        virtual double get_area_of_interest_shrink()const override
        {
          double value = 0.3;
          static const std::string name = prefix_ + "area_of_interest_shrink";
          get(name,value);
          return value;
        }
        /// filtering out all static objects not inside area of effect
        virtual double get_area_of_effect_shrink()const override
        {
          double value = 0.0;
          static const std::string name = prefix_ + "area_of_effect_shrink";
          get(name,value);
          return value;
        }
        /// filtering of precedence rules for worstcase maneuvers: @return if true activate filtering
        virtual bool get_worstcase_filter_precedence()const
        {
          bool value = true;
          static const std::string name = prefix_ + "worstcase_filter_precedence";
          get(name,value);
          return value;
        }
        /// filtering of tcd for worstcase maneuvers: @return if true activate filtering
        virtual bool get_worstcase_filter_tcd()const
        {
          bool value = true;
          static const std::string name = prefix_ + "worstcase_filter_tcd";
          get(name,value);
          return value;
        }
        /// returns prediction strategy: 0 width of object, 1 width of road, 2 width of object-> width of road
        virtual int get_setbased_prediction_strategy()const 
        {
          int value=0;
          static const std::string name = prefix_ + "setbased_prediction_strategy";
          get(name,value);
          return value;
        }
        /// returns maximum width for a prediction
        virtual double get_prediction_width_ub()const
        {
          double value = 10.0;
          static const std::string name = prefix_ + "prediction_width_ub";
          get(name,value);
          return value;
        }
        /// returns the minimum width for a prediction
        virtual double get_prediction_width_lb()const 
        {
          double value = 0.5;
          static const std::string name = prefix_ + "prediction_width_lb";
          get(name,value);
          return value;
        }

      };
    }
  }
}