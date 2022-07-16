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
 *   Thomas Lobig
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
    class PlotConflicts
    {
      private:
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;

      std::unordered_set<std::string> plot_tags_old_;
      std::unordered_set<std::string> plot_tags_current_;

      DLR_TS::PlotLab::AFigureStub* figure_;
      unsigned long long counter_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      adore::env::AFactory::TOCPredictionSetReader* conflict_set_reader_;

      double t_;
      double t_prediction_max_;

      adore::PLOT::PredictionConfig::prediction_config config_;


      public:
      PlotConflicts(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix)
      {
        counter_=0;
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        figure_ = figure;      
        prefix_ = prefix;
        t_ = 0;
        t_prediction_max_ = 10.0; // TODO: reconsidder where to get this max from, currently a plot property
        config_.active_ = true;
        config_.r_.start_ = 1.0;
        config_.r_.end_ = 0.0;
        config_.g_.start_ = 0.0;
        config_.g_.end_ = 1.0;
        config_.b_.start_ = 0.0;
        config_.b_.end_ = 0.0;

        conflict_set_reader_ = adore::env::EnvFactoryInstance::get()->getConflictSetReader();
      }

      ~PlotConflicts()
       {
           delete positionReader_;
           delete conflict_set_reader_;
       }

      void run()
      {

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          t_ = position_.getTime();
        }

        if(config_.active_ && conflict_set_reader_->hasUpdate())
        {
          plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
          plot_tags_current_.clear();

          std::stringstream tag;
          tag << prefix_ << "/conflict";
          adore::env::OccupancyCylinderPredictionSet conflict_set;
          conflict_set_reader_->getData(conflict_set);
          PLOT::plotPredictionSet(conflict_set,t_,t_prediction_max_,config_,tag.str(),figure_,plot_tags_current_);

          for(auto s:plot_tags_old_)
          {
            if(plot_tags_current_.find(s)==plot_tags_current_.end())
            {
              figure_->erase(s);
            }
          }
         }

        
      }


    };
  }
}