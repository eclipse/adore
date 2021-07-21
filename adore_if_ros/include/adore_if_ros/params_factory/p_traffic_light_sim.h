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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <adore/params/ap_traffic_light_sim.h>

namespace adore
{
  namespace if_ROS
  {
    namespace params
    {
      class PTrafficLightSim:public adore::params::APTrafficLightSim
      {
        private:
        ros::NodeHandle n_;
        std::string prefix_;

        template <class T> 
        T get(const std::string & name)const
        {
            if (!n_.hasParam(name))
            {
            ROS_INFO_STREAM("No parameter named " << name);
            }
            T result;
            n_.getParamCached(name,result);
            return result;
        }

        public:

        PTrafficLightSim(ros::NodeHandle n,std::string prefix) :n_(n),prefix_(prefix)
        {
        prefix_ = prefix + "traffic_light_sim/";
        }
        //visibility radius of the map provider
        virtual int getRedDuration()const override
        {
          static const std::string name = prefix_ + "r";
          return get<int>(name);
        }
        virtual int getGreenDuration()const override
        {
          static const std::string name = prefix_ + "g";
          return get<int>(name);
        }
        virtual int getRedYellowDuration()const override
        {
          static const std::string name = prefix_ + "ry";
          return get<int>(name);
        }
        virtual int getYellowDuration()const override
        {
          static const std::string name = prefix_ + "y";
          return get<int>(name);
        }
        virtual std::string getStartState()const override
        {
          static const std::string name = prefix_ + "start_state";
          return get<std::string>(name);
        }
        virtual bool getIsProbeMode()const override
        {
          static const std::string name = prefix_ + "probe";
          return get<bool>(name);
        }
      };
    }
  }
}