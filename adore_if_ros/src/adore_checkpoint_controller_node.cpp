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
 *   Daniel Heß - a ros node, which executes checkpoint_controller
 ********************************************************************************/

#include <adore_if_ros/baseapp.h>
#include <adore/apps/checkpoint_controller.h>


namespace adore
{
  namespace if_ROS
  {  
    class CheckpointControllerNode : public Baseapp
    {
    private:
      adore::apps::CheckpointController* cpc_;
    public:
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        cpc_ = new adore::apps::CheckpointController();
        std::string filename;
        getParam("PARAMS/checkpoints/filename", filename);
        if(filename.length()>0)cpc_->loadFromFile(filename);
        else std::cout<<nodename<<" could not load data: No filename provided"<<std::endl;

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::CheckpointController::run,cpc_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::CheckpointControllerNode node;
    node.init(argc, argv, 2.0, "adore_checkpoint_controller_node");
    node.run();
    return 0;
}