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
 *   Daniel Heß - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/params/ap_vehicle.h>
#include <adore/apps/if_plotlab/plot_prediction.h>
#include <adore/apps/if_plotlab/prediction_config.h>
#include <adore/env/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <unordered_set>
#include <string>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a plot module for handling prediction plots
     * 
     */
    class PlotPredictionsMinimal
    {
      private:
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;

      std::unordered_set<std::string> plot_tags_old_worstcase_;
      std::unordered_set<std::string> plot_tags_current_worstcase_;

      std::unordered_set<std::string> plot_tags_old_expected_;
      std::unordered_set<std::string> plot_tags_current_expected_;

      std::unordered_set<std::string> plot_tags_old_desired_;
      std::unordered_set<std::string> plot_tags_current_desired_;

      std::unordered_set<std::string> plot_tags_old_static_;
      std::unordered_set<std::string> plot_tags_current_static_;
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      unsigned long long counter_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      adore::env::OccupancyCylinderPredictionSet worst_case_predictions_;
      adore::env::OccupancyCylinderPredictionSet expected_predictions_;
      adore::env::OccupancyCylinderPredictionSet desired_predictions_;
      adore::env::OccupancyCylinderPredictionSet static_obstacles_;

      adore::env::AFactory::TOCPredictionSetReader* worst_case_prediction_reader_;
      adore::env::AFactory::TOCPredictionSetReader* desired_prediction_reader_;
      adore::env::AFactory::TOCPredictionSetReader* expected_prediction_reader_;
      adore::env::AFactory::TOCPredictionSetReader* staticObstaclesReader_;

      int plotCount_;
      int eraseCount_;
      double fov_width_;
      double fov_height_;
      // int urlPreset_;
      double t_;
      double t_prediction_max_;

      adore::PLOT::PredictionConfig config_;

      public:
      // FancyBird(DLR_TS::PlotLab::AFigureStub* figure,adore::params::APVehicle* pvehicle, adore::params::APMapProvider* pmap,std::string prefix,const adore::PLOT::FancyBirdConfig & config, const adore::PLOT::GeoTilesConfig geoTiles_config)
      PlotPredictionsMinimal(DLR_TS::PlotLab::AFigureStub* figure, std::string prefix,const adore::PLOT::PredictionConfig & config)
      {
        counter_=0;
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        figure_ = figure;      
        prefix_ = prefix;
        t_ = 0;
        t_prediction_max_ = config_.t_prediction_max; // TODO: reconsidder where to get this max from, currently a plot property
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        config_ = config;

        worst_case_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getWorstCasePredictionSetReader();
        desired_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getDesiredPredictionSetReader();
        expected_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getExpectedPredictionSetReader();
        staticObstaclesReader_ = adore::env::EnvFactoryInstance::get()->getStaticObstaclesPredictionSetReader();
      }

      ~PlotPredictionsMinimal()
       {
        //  geoTiles_->~GeoTiles();
       }

      void run()
      {

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          t_ = position_.getTime();
        }

        /* Worst Case Prediction Feed */
        if(config_.worst_case_.active_ and worst_case_prediction_reader_->hasUpdate())
        {
          plot_tags_old_worstcase_.insert(plot_tags_current_worstcase_.begin(),plot_tags_current_worstcase_.end());
          plot_tags_current_worstcase_.clear();

          std::stringstream tag;
          tag << prefix_ << "/wcp";
          worst_case_prediction_reader_->getData(worst_case_predictions_);
          PLOT::plotPredictionSetMinimal(worst_case_predictions_,0.5,config_.worst_case_,tag.str(),figure_,plot_tags_current_worstcase_);

          for(auto s:plot_tags_old_worstcase_)
          {
            if(plot_tags_current_worstcase_.find(s)==plot_tags_current_worstcase_.end())
            {
              figure_->erase(s);
            }
          }
         }

        // /* Expected Prediction Feed */
        if(config_.expected_.active_ and expected_prediction_reader_->hasUpdate())
        {
          plot_tags_old_expected_.insert(plot_tags_current_expected_.begin(),plot_tags_current_expected_.end());
          plot_tags_current_expected_.clear();

          std::stringstream tag;
          tag << prefix_ << "/ecp";
          expected_prediction_reader_->getData(expected_predictions_);
          PLOT::plotPredictionSetMinimal(expected_predictions_,0.0,config_.expected_,tag.str(),figure_,plot_tags_current_expected_);

          for(auto s:plot_tags_old_expected_)
          {
            if(plot_tags_current_expected_.find(s)==plot_tags_current_expected_.end())
            {
              figure_->erase(s);
            }
          }
        }

        // /* Desired Prediction Feed */
        if(config_.desired_.active_ and desired_prediction_reader_->hasUpdate())
        {
          plot_tags_old_desired_.insert(plot_tags_current_desired_.begin(),plot_tags_current_desired_.end());
          plot_tags_current_desired_.clear();

          std::stringstream tag;
          tag << prefix_ << "/dcp";
          desired_prediction_reader_->getData(desired_predictions_);
          PLOT::plotPredictionSetMinimal(desired_predictions_,0.25,config_.expected_,tag.str(),figure_,plot_tags_current_desired_);

          for(auto s:plot_tags_old_desired_)
          {
            if(plot_tags_current_desired_.find(s)==plot_tags_current_desired_.end())
            {
              figure_->erase(s);
            }
          }
        }

        // /* static obstacles Feed */
        if(config_.static_.active_ and staticObstaclesReader_->hasUpdate())
        {
          plot_tags_old_static_.insert(plot_tags_current_static_.begin(),plot_tags_current_static_.end());
          plot_tags_current_static_.clear();

          std::stringstream tag;
          tag << prefix_ << "/sop";
          staticObstaclesReader_->getData(static_obstacles_);
          PLOT::plotPredictionSet(static_obstacles_,t_,t_prediction_max_,config_.static_,tag.str(),figure_,plot_tags_current_static_);

          for(auto s:plot_tags_old_static_)
          {
            if(plot_tags_current_static_.find(s)==plot_tags_current_static_.end())
            {
              figure_->erase(s);
            }
          }
        }
        
      }


    };
  }
}