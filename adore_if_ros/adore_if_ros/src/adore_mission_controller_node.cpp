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
 *   Jan Lauermann - a ros node, which executes mission_controller
 ********************************************************************************/

#include <adore_if_ros/baseapp.h>
#include <adore/apps/mission_controller.h>


namespace adore
{
  namespace if_ROS
  {  
    class MissionControllerNode : public Baseapp
    {
    private:
      adore::apps::MissionController* mc_;
    public:
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        mc_ = new adore::apps::MissionController(this->getFactory<FUN_Factory>(),this->getParamsFactory(""));

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::MissionController::run,mc_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::MissionControllerNode node;
    node.init(argc, argv, 2.0, "adore_mission_controller_node");
    node.run();
    return 0;
}