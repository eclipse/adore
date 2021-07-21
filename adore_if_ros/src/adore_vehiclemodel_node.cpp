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


#include <adore_if_ros/simfactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore/apps/vehiclemodel.h>
#include <adore/sim/schedulernotificationmanager.h>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <adore_if_ros/baseapp.h>


namespace adore
{
  namespace if_ROS
  {  
    class VehicleModelNode : public Baseapp
    {
      public:
      adore::apps::VehicleModel* vm_;
      VehicleModelNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        int simulationID = 0;
        int v2xStationID = 0;
        getParam("simulationID",simulationID);
        getParam("v2xStationID",v2xStationID);
        vm_ = new adore::apps::VehicleModel(getFactory<SIM_Factory>(),getParamsFactory(""),simulationID,v2xStationID);
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::VehicleModel::run,vm_));
        Baseapp::addTimerCallback(run_fcn);
      }
      void setAutomaticControl(bool value)
      {
        vm_->setAutomaticControl(value);
      }
      void setCheckpointClearance()
      {
        vm_->setCheckpointClearance();
      }
    };
  }
}

adore::if_ROS::VehicleModelNode vmn;
bool automatic_control;
bool terminated;

void kbinput()
{
  while(!terminated)
  {
    int c = std::cin.get();
    if( c == 49) 
    {
      automatic_control=!automatic_control;
      vmn.setAutomaticControl(automatic_control);
      std::cout<<"control="<<(automatic_control?"automatic":"manual")<<std::endl;
    }
    if( c == 50 )
    {
      vmn.setCheckpointClearance();
      std::cout<<"user pressed checkpoint clearance button"<<std::endl;
    }
  }
}


int main(int argc,char **argv)
{
    automatic_control = true;
    for(int i=0;i<argc;i++)if(strcmp(argv[i],"manual_control")==0)automatic_control = false;
    std::cout<<"adore_vehiclemodel_node"<<std::endl;
    std::cout<<"control="<<(automatic_control?"automatic":"manual")<<std::endl;
    std::cout<<"press 1 + Enter to switch manual/automatic"<<std::endl;
    std::cout<<"press 2 + Enter to confirm checkpoint"<<std::endl;
    std::thread kbinput_thread(kbinput);
    vmn.init(argc, argv, 100.0, "adore_vehiclemodel_node");
    vmn.setAutomaticControl(automatic_control);    
    vmn.run();
    return 0;
}
