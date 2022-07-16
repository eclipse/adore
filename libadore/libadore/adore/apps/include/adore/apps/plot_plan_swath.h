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
#include <adore/apps/if_plotlab/plot_prediction.h>
#include <adore/fun/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <string>
#include <unordered_map>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a plot module for plotting the planning result swaths
     * 
     */
    class PlotPlanSwath
    {
        private:      
        DLR_TS::PlotLab::AFigureStub* figure_;
        std::string prefix_;
        adore::mad::AFeed<adore::fun::PlanningResult>* planning_result_feed_;
        std::unordered_map<std::string,int> tag_to_iteration_;
        int max_iteration_;
        bool plot_combined_;
        bool plot_nominal_;
        PLOT::PredictionConfig::prediction_config cb_config_;
        PLOT::PredictionConfig::prediction_config nom_config_;


      public:

      PlotPlanSwath(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix)
      {
        figure_ = figure;      
        prefix_ = prefix;
        max_iteration_ = 0;
        plot_combined_ = true;
        plot_nominal_ = true;
        cb_config_.r_.start_ = 1.0;
        cb_config_.r_.end_ = 0.0;
        cb_config_.g_.start_ = 0.0;
        cb_config_.g_.end_ = 1.0;
        cb_config_.b_.start_ = 0.0;
        cb_config_.b_.end_ = 0.0;
        nom_config_.r_.start_ = 1.0;
        nom_config_.r_.end_ = 0.0;
        nom_config_.g_.start_ = 0.0;
        nom_config_.g_.end_ = 0.0;
        nom_config_.b_.start_ = 1.0;
        nom_config_.b_.end_ = 1.0;


        planning_result_feed_ = adore::fun::FunFactoryInstance::get()->getPlanningResultFeed();
      }


      void run()
      {
        std::unordered_set<std::string> plot_tags;

        while(planning_result_feed_->hasNext())
        {
            adore::fun::PlanningResult result;
            planning_result_feed_->getNext(result);
            max_iteration_ = std::max(max_iteration_,result.iteration);
            if(plot_combined_ && result.combined_maneuver_valid
            && result.combined_maneuver_swath.getLevel(0).size()>0)
            {
              std::stringstream tag;
              tag << prefix_ << "/PlanningResult/cb/"<<result.id<<"/";
              std::string tagstr = tag.str();
              double t0 = result.combined_maneuver_swath.getLevel(0).begin()->second.t0_;
              double t1 = result.combined_maneuver_swath.getLevel(0).rbegin()->second.t1_;
              PLOT::plotCylinderTree(result.combined_maneuver_swath,t0,t1,cb_config_,tagstr,figure_,plot_tags);
              tag_to_iteration_[tagstr] = result.iteration;
            }
            if(plot_nominal_ && result.combined_maneuver_valid
            && result.nominal_maneuver_swath.getLevel(0).size()>0)
            {
              std::stringstream tag;
              tag << prefix_ << "/PlanningResult/nom/"<<result.id<<"/";
              std::string tagstr = tag.str();
              double t0 = result.nominal_maneuver_swath.getLevel(0).begin()->second.t0_;
              double t1 = result.nominal_maneuver_swath.getLevel(0).rbegin()->second.t1_;
              PLOT::plotCylinderTree(result.nominal_maneuver_swath,t0,t1,nom_config_,tagstr,figure_,plot_tags);
              tag_to_iteration_[tagstr] = result.iteration;
            }
        }
        for(auto& it:tag_to_iteration_)
        {
          if(it.second<max_iteration_-1)
          {
            figure_->erase_similar(it.first);
            tag_to_iteration_[it.first] = std::numeric_limits<int>::max();
          }
        }
      }    
    };
  }
}