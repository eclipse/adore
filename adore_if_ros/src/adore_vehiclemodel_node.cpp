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
        getParam("simulationID",simulationID);
        vm_ = new adore::apps::VehicleModel(getFactory<SIM_Factory>(),getParamsFactory(""),simulationID);
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::VehicleModel::run,vm_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}
int main(int argc,char **argv)
{
    adore::if_ROS::VehicleModelNode vmn;
    vmn.init(argc, argv, 100.0, "adore_vehiclemodel_node");
    vmn.run();
    return 0;
}
