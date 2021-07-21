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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore/params/ap_sensor_model.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PSensorModel:public adore::params::APSensorModel, ROSParam
        {
          public:
          PSensorModel(ros::NodeHandle n,std::string prefix)
          :ROSParam(n,prefix + "SensorModel/")//using SensorModel instead of Sensor to discriminate with real sensor parameters
          {
          }
          ///maximum sensor range for object detection in a generalized sensor setting
          virtual double get_objectDetectionRange()const override
          {
            double value = 1000.0;
            static const std::string name = prefix_ + "objectDetectionRange";
            get(name,value);
            return value;
          }
			    ///time after which object detections are discarded
          virtual double get_objectDiscardAge()const override
          {
            double value = 1.0;
            static const std::string name = prefix_ + "objectDiscardAge";
            get(name,value);
            return value;
          }
       };
    }
  }
}