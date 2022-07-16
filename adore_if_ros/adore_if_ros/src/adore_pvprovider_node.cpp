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
 *   Reza Dariani   platoon view provider
 ********************************************************************************/

#include <adore_if_ros/baseapp.h>
#include <adore/apps/platoon_view_provider.h>


namespace adore
{
  namespace if_ROS
  {  
    class PVProviderNode : public Baseapp
    {
      public:
      adore::apps::PlatoonViewProvider* pvp_;
      PVProviderNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        pvp_ = new adore::apps::PlatoonViewProvider();

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::PlatoonViewProvider::run,pvp_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PVProviderNode node;
    node.init(argc, argv, 20.0, "adore_pvprovider_node");
    node.run();
    return 0;
}