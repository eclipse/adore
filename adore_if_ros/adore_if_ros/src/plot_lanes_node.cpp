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

#include <adore/apps/plot_lanes.h>
#include <adore/apps/if_plotlab/laneplot_config.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotLanesNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::PlotLanes* app_;
      PlotLanesNode(){}
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
 
        adore::PLOT::LanePlotConfig laneplot_config;

        // default settings if nothing else is set:
        std::string outer_border = "LineColor=0.6,0.6,0.6;LineWidth=2.3";
        laneplot_config.border_outer_left_plotoptions = "";
        laneplot_config.border_outer_right_plotoptions = "";
        laneplot_config.lane_fill_driveable_plotoptions = "FillColor=0.3,0.3,0.3;LineColor=0.25,0.25,0.25;LineWidth=0.5";
        laneplot_config.lane_fill_emergency_plotoptions = "FillColor=0.9,0.8,0.8;LineColor=0.8,0.7,0.7;LineWidth=1.8";
        laneplot_config.lane_fill_other_plotoptions = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";
        laneplot_config.lane_fill_default_plotoptions = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";
        laneplot_config.setpoint_plotoptions = "LineWidth=3;LineColor=0.9,0.1,0.1";
        laneplot_config.plot_traffic_lights = false;
        laneplot_config.plot_emergency_lane = false;
        laneplot_config.plot_other_lane = false;
        laneplot_config.plot_drive_lane = true;
        // TODO the following seems to be missing: laneplot_config.left_border
        std::string left_border = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";


        // try and read parameters, ROS will not overwrite the value if it is not available
        getParam("plotoptions/outer_border",outer_border);
        getParam("plotoptions/leftmost_border",laneplot_config.border_outer_left_plotoptions);
        getParam("plotoptions/rightmost_border",laneplot_config.border_outer_right_plotoptions);
        getParam("plotoptions/drive_lane",laneplot_config.lane_fill_driveable_plotoptions);
        getParam("plotoptions/emergency_lane",laneplot_config.lane_fill_emergency_plotoptions);
        getParam("plotoptions/other_lane",laneplot_config.lane_fill_other_plotoptions);+
        getParam("plotoptions/default_lane",laneplot_config.lane_fill_default_plotoptions);
        getParam("plotoptions/setpoint",laneplot_config.setpoint_plotoptions);
        getParam("plotoptions/plot_traffic_lights",laneplot_config.plot_traffic_lights);
        getParam("plotoptions/plot_emergency_lane",laneplot_config.plot_emergency_lane);
        getParam("plotoptions/plot_other_lane",laneplot_config.plot_other_lane);
        getParam("plotoptions/plot_drive_lane",laneplot_config.plot_drive_lane );

        // reevaluate if some options remained unset and use other defaults when needed
        if (laneplot_config.border_outer_left_plotoptions == "" )
        {
            laneplot_config.border_outer_left_plotoptions = outer_border;
        }
        if (laneplot_config.border_outer_right_plotoptions == "")
        {
            laneplot_config.border_outer_right_plotoptions = outer_border;
        }
        
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotLanes(figure,
                                            ss.str(),
                                            laneplot_config); //,
                                            // geoTiles_config);
        //@TODO activate plotting / management of borderset --> extra visualization module in params
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotLanes::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotLanesNode appnode;    
    appnode.init(argc, argv, 10.0, "plot_lanes_node");
    appnode.run();
    return 0;
}
