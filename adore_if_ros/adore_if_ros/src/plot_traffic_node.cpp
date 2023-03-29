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

#include <adore/apps/plot_traffic.h>
// #include <adore/apps/if_plotlab/fancy_config.h>
// #include <adore/apps/if_plotlab/geoTiles_config.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotTrafficNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::PlotTraffic* app_;
      PlotTrafficNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
        getParam("simulationID",simulationID);

        bool debug_traffic_ids = 0;
        getParam("debugTrafficIDs",debug_traffic_ids);
 
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotTraffic(figure,
                                            ss.str(),debug_traffic_ids);
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotTraffic::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotTrafficNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_traffic_node");
    fbn.run();
    return 0;
}
