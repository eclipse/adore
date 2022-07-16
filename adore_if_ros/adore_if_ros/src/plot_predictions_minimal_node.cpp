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
*   Daniel Heß
********************************************************************************/

#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/funfactory.h>
#include <adore/apps/plot_predictions_minimal.h>
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
    class PlotPredictionsMinimalNode : public Baseapp
    {
      public:
      adore::apps::PlotPredictionsMinimal* pb_;
      PlotPredictionsMinimalNode(){}
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
        getParam("plotoptions/prediction/dcp/active",  config_.desired_.active_);
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

        /* static prediction */
        /* bool */
        getParam("plotoptions/prediction/scp/active",  config_.static_.active_);
        /* double I=[0,1] */
        getParam("plotoptions/prediction/scp/r_start", config_.static_.r_.start_);
        getParam("plotoptions/prediction/scp/r_end",   config_.static_.r_.end_);
        getParam("plotoptions/prediction/scp/g_start", config_.static_.g_.start_);
        getParam("plotoptions/prediction/scp/g_end",   config_.static_.g_.end_);
        getParam("plotoptions/prediction/scp/b_start", config_.static_.b_.start_);
        getParam("plotoptions/prediction/scp/b_end",   config_.static_.b_.end_);

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

        getParam("plotoptions/prediction/t_prediction_max", config_.t_prediction_max);
        
        // adore::PLOT::GeoTilesConfig geoTiles_config;
        // getParam("plotoptions/tiles/base_url",geoTiles_config.base_url);
        // getParam("plotoptions/tiles/width_meters",geoTiles_config.tile_width_meters);

        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        pb_ = new adore::apps::PlotPredictionsMinimal(figure,
                                            new adore::params::APVehicleDummy(),
                                            ss.str(),
                                            config_);
        //@TODO activate plotting / management of borderset --> extra visualization module in params
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotPredictionsMinimal::run,pb_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotPredictionsMinimalNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_predictions_node");
    fbn.run();
    return 0;
}