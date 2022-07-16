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
*   Daniel Heß
********************************************************************************/

#include <adore/apps/plot_gaps.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class ThisNode : public Baseapp
    {
      public:
      adore::apps::PlotGaps* app_;
      ThisNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        int simulationID = 0;
        getParam("simulationID",simulationID);
 
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotGaps(ss.str());
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotGaps::update,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::ThisNode node;    
    node.init(argc, argv, 10.0, "plot_gaps_node");
    node.run();
    return 0;
}
