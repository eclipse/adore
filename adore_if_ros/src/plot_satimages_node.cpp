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

#include <adore/apps/plot_satimages.h>
#include <adore/apps/if_plotlab/geoTiles_config.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotSatImagesNode : public Baseapp
    {
      public:
      adore::apps::PlotSatImages* app_;
      PlotSatImagesNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
        getParam("simulationID",simulationID);
        
        adore::PLOT::GeoTilesConfig geoTiles_config;
        getParam("plotoptions/tiles/base_url",geoTiles_config.base_url);
        getParam("plotoptions/tiles/width_meters",geoTiles_config.tile_width_meters);

        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotSatImages(figure,
                                            ss.str(),
                                            geoTiles_config);
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotSatImages::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotSatImagesNode appnode;    
    appnode.init(argc, argv, 10.0, "plot_satimages_node");
    appnode.run();
    return 0;
}
