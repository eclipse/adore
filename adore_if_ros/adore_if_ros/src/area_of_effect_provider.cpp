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

#include <adore_if_ros/baseapp.h>
#include <adore/apps/area_of_effect_provider.h>

namespace adore
{
  namespace if_ROS
  {  
    class AreaOfEffectProviderNode : public Baseapp
    {
      public:
      adore::apps::AreaOfEffectProvider* app_;
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        app_ = new adore::apps::AreaOfEffectProvider();
        std::function<void()> run_fcn(std::bind(&adore::apps::AreaOfEffectProvider::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::AreaOfEffectProviderNode node;
    node.init(argc, argv, 10, "area_of_effect_provider");
    node.run();
    return 0;
}