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

#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/funfactory.h>
#include <adore/apps/borderbird.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/params/ap_vehicle_dummy.h>
#include <adore/params/ap_map_provider_dummy.h>
#include <adore_if_ros/baseapp.h>

namespace adore
{
  namespace if_ROS
  {  
    class BorderBirdNode : public Baseapp
    {
      public:
      adore::apps::BorderBird* bb_;
      BorderBirdNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();

        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
        getParam("simulationID",simulationID);
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        bb_ = new adore::apps::BorderBird(getFactory<ENV_Factory>(),getFactory<FUN_Factory>(),figure,
                                        new adore::params::APVehicleDummy(),
                                        new adore::params::APMapProviderDummy(),
                                        ss.str());
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::BorderBird::run,bb_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}
int main(int argc,char **argv)
{
    adore::if_ROS::BorderBirdNode bbn;
    bbn.init(argc, argv, 10.0, "adore_borderbird_node");
    bbn.run();
    return 0;
}
