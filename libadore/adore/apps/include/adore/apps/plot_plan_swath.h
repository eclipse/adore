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
#include <unordered_set>


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
        std::unordered_set<std::string> plot_tags_old_;
        std::unordered_set<std::string> plot_tags_current_;
      

      public:

      PlotPlanSwath(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix)
      {
        figure_ = figure;      
        prefix_ = prefix;

        planning_result_feed_ = adore::fun::FunFactoryInstance::get()->getPlanningResultFeed();
      }


      void run()
      {
        plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
        plot_tags_current_.clear();

        while(planning_result_feed_->hasNext())
        {
            adore::fun::PlanningResult result;
            planning_result_feed_->getNext(result);
            if(result.combined_maneuver_swath.getLevel(0).size()>0)
            {
                std::stringstream tag;
                tag << prefix_ << "/PlanningResult/"<<result.id<<"/";
                PLOT::PredictionConfig::prediction_config config;
                double t0 = result.combined_maneuver_swath.getLevel(0).begin()->second.t0_;
                double t1 = result.combined_maneuver_swath.getLevel(0).rbegin()->second.t1_;
                PLOT::plotCylinderTree(result.combined_maneuver_swath,t0,t1,config,tag.str(),figure_,plot_tags_current_);
            }
        }
      }    
    };
  }
}