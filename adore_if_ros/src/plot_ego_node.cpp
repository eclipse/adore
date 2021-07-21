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
*   Thomas Lobig
********************************************************************************/

#include <adore/apps/plot_ego.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/params/ap_vehicle_dummy.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotEgoNode : public Baseapp
    {
      public:
      adore::apps::PlotEgo* app_;
      PlotEgoNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
        getParam("simulationID",simulationID);

        int followMode = 0;
        getParam("plotoptions/followMode",followMode);
 
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotEgo(figure,
                                            new adore::params::APVehicleDummy(),//@TODO: why is vehicle dummy used?
                                            ss.str(),
                                            followMode);
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotEgo::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotEgoNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_ego_node");
    fbn.run();
    return 0;
}
