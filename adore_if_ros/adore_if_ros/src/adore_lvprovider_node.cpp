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
 *   Daniel Heß - a process, which computes lane following geometry
 ********************************************************************************/

#include <adore_if_ros/baseapp.h>
#include <adore/apps/lane_view_provider.h>


namespace adore
{
  namespace if_ROS
  {  
    class LVProviderNode : public Baseapp
    {
      public:
      adore::apps::LaneViewProvider* lvp_;
      LVProviderNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        lvp_ = new adore::apps::LaneViewProvider();

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::LaneViewProvider::run,lvp_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::LVProviderNode node;
    node.init(argc, argv, 10.0, "adore_lvprovider_node");
    node.run();
    return 0;
}