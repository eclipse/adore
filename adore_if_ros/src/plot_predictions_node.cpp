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

#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/funfactory.h>
#include <adore/apps/plot_predictions.h>
#include <adore/apps/if_plotlab/prediction_config.h>
// #include <adore/apps/if_plotlab/geoTiles_config.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/params/ap_vehicle_dummy.h>
// #include <adore/params/ap_map_provider_dummy.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotPredictionsNode : public Baseapp
    {
      public:
      adore::apps::PlotPredictions* pb_;
      PlotPredictionsNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
 
        adore::PLOT::PredictionConfig config_;
        getParam("simulationID",simulationID);

        // default settings if nothing else is set:
        // std::string outer_border = "LineColor=0.6,0.6,0.6;LineWidth=2.3";
        // fancy_config.followMode = 0;
        // fancy_config.border_outer_left_plotoptions = "";
        // fancy_config.border_outer_right_plotoptions = "";
        // fancy_config.lane_fill_driveable_plotoptions = "FillColor=0.3,0.3,0.3;LineColor=0.25,0.25,0.25;LineWidth=0.5";
        // fancy_config.lane_fill_emergency_plotoptions = "FillColor=0.9,0.8,0.8;LineColor=0.8,0.7,0.7;LineWidth=1.8";
        // fancy_config.lane_fill_other_plotoptions = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";
        // fancy_config.lane_fill_default_plotoptions = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";
        // fancy_config.setpoint_plotoptions = "LineWidth=3;LineColor=0.9,0.1,0.1";
        // fancy_config.plot_traffic_lights = false;
        // fancy_config.plot_emergency_lane = false;
        // fancy_config.plot_other_lane = false;
        // fancy_config.plot_drive_lane = true;
        // // TODO the following seems to be missing: fancy_config.left_border
        // std::string left_border = "FillColor=0.7,0.7,0.7;LineColor=0.6,0.6,0.6;LineWidth=1.8";


        // int followMode = 0;
        // // try and read parameters, ROS will not overwrite the value if it is not available
        // getParam("plotoptions/followMode",followMode);
        // fancy_config.followMode = followMode != 0;
        // getParam("plotoptions/outer_border",outer_border);
        // getParam("plotoptions/leftmost_border",fancy_config.border_outer_left_plotoptions);
        // getParam("plotoptions/rightmost_border",fancy_config.border_outer_right_plotoptions);
        // getParam("plotoptions/drive_lane",fancy_config.lane_fill_driveable_plotoptions);
        // getParam("plotoptions/emergency_lane",fancy_config.lane_fill_emergency_plotoptions);
        // getParam("plotoptions/other_lane",fancy_config.lane_fill_other_plotoptions);+
        // getParam("plotoptions/default_lane",fancy_config.lane_fill_default_plotoptions);
        // getParam("plotoptions/setpoint",fancy_config.setpoint_plotoptions);
        // getParam("plotoptions/plot_traffic_lights",fancy_config.plot_traffic_lights);
        // getParam("plotoptions/plot_emergency_lane",fancy_config.plot_emergency_lane);
        // getParam("plotoptions/plot_other_lane",fancy_config.plot_other_lane);
        // getParam("plotoptions/plot_drive_lane",fancy_config.plot_drive_lane );

        // // reevaluate if some options remained unset and use other defaults when needed
        // if (fancy_config.border_outer_left_plotoptions == "" )
        // {
        //     fancy_config.border_outer_left_plotoptions = outer_border;
        // }
        // if (fancy_config.border_outer_right_plotoptions == "")
        // {
        //     fancy_config.border_outer_right_plotoptions = outer_border;
        // }

                /* worst case prediction */
        /* bool */
        getParam("plotoptions/prediction/wcp/active",  config_.worst_case_.active_);
        /* double I=[0,1] */
        getParam("plotoptions/prediction/wcp/r_start", config_.worst_case_.r_.start_);
        getParam("plotoptions/prediction/wcp/r_end",   config_.worst_case_.r_.end_);
        getParam("plotoptions/prediction/wcp/g_start", config_.worst_case_.g_.start_);
        getParam("plotoptions/prediction/wcp/g_end",   config_.worst_case_.g_.end_);
        getParam("plotoptions/prediction/wcp/b_start", config_.worst_case_.b_.start_);
        getParam("plotoptions/prediction/wcp/b_end",   config_.worst_case_.b_.end_);
        
        /* desired prediction */
        /* bool */
        getParam("plotoptions/prediction/wcp/active",  config_.desired_.active_);
        /* double I=[0,1] */
        getParam("plotoptions/prediction/dcp/r_start", config_.desired_.r_.start_);
        getParam("plotoptions/prediction/dcp/r_end",   config_.desired_.r_.end_);
        getParam("plotoptions/prediction/dcp/g_start", config_.desired_.g_.start_);
        getParam("plotoptions/prediction/dcp/g_end",   config_.desired_.g_.end_);
        getParam("plotoptions/prediction/dcp/b_start", config_.desired_.b_.start_);
        getParam("plotoptions/prediction/dcp/b_end",   config_.desired_.b_.end_);

        /* expected prediction */
        /* bool */
        getParam("plotoptions/prediction/ecp/active",  config_.expected_.active_);
        /* double I=[0,1] */
        getParam("plotoptions/prediction/ecp/r_start", config_.expected_.r_.start_);
        getParam("plotoptions/prediction/ecp/r_end",   config_.expected_.r_.end_);
        getParam("plotoptions/prediction/ecp/g_start", config_.expected_.g_.start_);
        getParam("plotoptions/prediction/ecp/g_end",   config_.expected_.g_.end_);
        getParam("plotoptions/prediction/ecp/b_start", config_.expected_.b_.start_);
        getParam("plotoptions/prediction/ecp/b_end",   config_.expected_.b_.end_);

        /* ego prediction */
        /* bool */
        getParam("plotoptions/prediction/ego/active",  config_.ego_.active_);
        /* double I=[0,1] */
        getParam("plotoptions/prediction/ego/r_start", config_.ego_.r_.start_);
        getParam("plotoptions/prediction/ego/r_end",   config_.ego_.r_.end_);
        getParam("plotoptions/prediction/ego/g_start", config_.ego_.g_.start_);
        getParam("plotoptions/prediction/ego/g_end",   config_.ego_.g_.end_);
        getParam("plotoptions/prediction/ego/b_start", config_.ego_.b_.start_);
        getParam("plotoptions/prediction/ego/b_end",   config_.ego_.b_.end_);

        double t_prediction_max = 5.0;
        getParam("plotoptions/prediction/t_prediction_max", config_.t_prediction_max);
        
        // adore::PLOT::GeoTilesConfig geoTiles_config;
        // getParam("plotoptions/tiles/base_url",geoTiles_config.base_url);
        // getParam("plotoptions/tiles/width_meters",geoTiles_config.tile_width_meters);

        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        pb_ = new adore::apps::PlotPredictions(figure,
                                            new adore::params::APVehicleDummy(),
                                            ss.str(),
                                            config_);
        //@TODO activate plotting / management of borderset --> extra visualization module in params
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotPredictions::run,pb_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotPredictionsNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_predictions_node");
    fbn.run();
    return 0;
}